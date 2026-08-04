// Copyright (C) 2026 Ian Engebrecht
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "ld2450.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ld2450";

#define FRAME_HEADER_0 0xAA
#define FRAME_HEADER_1 0xFF
#define FRAME_HEADER_2 0x03
#define FRAME_HEADER_3 0x00
#define FRAME_FOOTER_0 0x55
#define FRAME_FOOTER_1 0xCC

#define TARGET_BYTES 8
#define TARGET_COUNT 3
#define FRAME_DATA_LEN (TARGET_BYTES * TARGET_COUNT)
#define UART_BUF_SIZE 1024

#define MAX_ABS_X_MM 6000
#define MAX_ABS_Y_MM 8000

struct ld2450_dev {
    uart_port_t uart_num;
};

static int16_t decode_signed_le(const uint8_t *p)
{
    const uint16_t raw = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    const int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? (int16_t)(-mag) : mag;
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void parse_target(const uint8_t *p, ld2450_target_t *t)
{
    bool all_zero = true;
    for (int i = 0; i < TARGET_BYTES; ++i) {
        if (p[i] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        memset(t, 0, sizeof(*t));
        t->valid = false;
        return;
    }

    t->x_mm = decode_signed_le(p);
    t->y_mm = decode_signed_le(p + 2);
    t->speed_cm_s = decode_signed_le(p + 4);
    (void)read_u16_le(p + 6);

    const float dist =
        sqrtf((float)t->x_mm * (float)t->x_mm + (float)t->y_mm * (float)t->y_mm);
    t->distance_mm = (uint16_t)(dist > 65535.0f ? 65535.0f : dist);

    const int ax = t->x_mm < 0 ? -t->x_mm : t->x_mm;
    const int ay = t->y_mm < 0 ? -t->y_mm : t->y_mm;
    t->valid = (ax <= MAX_ABS_X_MM && ay <= MAX_ABS_Y_MM && t->distance_mm > 0);
}

esp_err_t ld2450_create(const ld2450_config_t *cfg, ld2450_dev_t **out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    ld2450_dev_t *dev = calloc(1, sizeof(*dev));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }
    dev->uart_num = cfg->uart_num;

    const uart_config_t uart_cfg = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg->uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        free(dev);
        return err;
    }
    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) {
        uart_driver_delete(cfg->uart_num);
        free(dev);
        return err;
    }

    const int tx = (cfg->tx_gpio < 0) ? UART_PIN_NO_CHANGE : cfg->tx_gpio;
    const int rx = cfg->rx_gpio;
    err = uart_set_pin(cfg->uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(cfg->uart_num);
        free(dev);
        return err;
    }

    ESP_LOGI(TAG, "UART%d @ %d RX=%d TX=%d", (int)cfg->uart_num, cfg->baud, rx, tx);
    *out = dev;
    return ESP_OK;
}

void ld2450_destroy(ld2450_dev_t *dev)
{
    if (!dev) {
        return;
    }
    uart_driver_delete(dev->uart_num);
    free(dev);
}

/* ── Configuration command infrastructure ─────────────────────────── */

#define CMD_HEADER_0 0xFD
#define CMD_HEADER_1 0xFC
#define CMD_HEADER_2 0xFB
#define CMD_HEADER_3 0xFA

#define CMD_FOOTER_0 0x04
#define CMD_FOOTER_1 0x03
#define CMD_FOOTER_2 0x02
#define CMD_FOOTER_3 0x01

#define ACK_TIMEOUT_MS 500

static esp_err_t send_raw(ld2450_dev_t *dev, const uint8_t *data, int len)
{
    int written = uart_write_bytes(dev->uart_num, data, len);
    if (written != len) return ESP_FAIL;
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t wait_ack(ld2450_dev_t *dev, uint16_t *out_status)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
    uint8_t b;
    int state = 0;
    uint8_t ack_buf[64];
    int ack_len = 0;

    while (xTaskGetTickCount() < deadline) {
        TickType_t remain = deadline - xTaskGetTickCount();
        if (remain == 0) remain = 1;
        int n = uart_read_bytes(dev->uart_num, &b, 1, remain);
        if (n != 1) continue;

        if (state < 4) {
            /* Hunt for command ACK header FD FC FB FA, skip data frames */
            const uint8_t expect[] = {CMD_HEADER_0, CMD_HEADER_1,
                                      CMD_HEADER_2, CMD_HEADER_3};
            state = (b == expect[state]) ? state + 1 : (b == CMD_HEADER_0 ? 1 : 0);
        } else {
            ack_buf[ack_len++] = b;
            if (ack_len >= 4 &&
                ack_buf[ack_len - 4] == CMD_FOOTER_0 &&
                ack_buf[ack_len - 3] == CMD_FOOTER_1 &&
                ack_buf[ack_len - 2] == CMD_FOOTER_2 &&
                ack_buf[ack_len - 1] == CMD_FOOTER_3) {
                uint16_t status = 0;
                if (ack_len >= 6)
                    status = (uint16_t)ack_buf[4] | ((uint16_t)ack_buf[5] << 8);
                if (out_status) *out_status = status;
                return (status == 0) ? ESP_OK : ESP_FAIL;
            }
            if (ack_len >= (int)sizeof(ack_buf)) return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static void force_end_config(ld2450_dev_t *dev)
{
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    send_raw(dev, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(dev->uart_num);
}

static esp_err_t enter_config(ld2450_dev_t *dev)
{
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xFF, 0x00,
        0x01, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    uart_flush_input(dev->uart_num);
    vTaskDelay(pdMS_TO_TICKS(30));
    uart_flush_input(dev->uart_num);

    esp_err_t err = send_raw(dev, cmd, sizeof(cmd));
    if (err != ESP_OK) return err;
    return wait_ack(dev, NULL);
}

static esp_err_t end_config(ld2450_dev_t *dev)
{
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    esp_err_t err = send_raw(dev, cmd, sizeof(cmd));
    if (err != ESP_OK) return err;
    esp_err_t ack = wait_ack(dev, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(dev->uart_num);
    return ack;
}

static esp_err_t send_config_cmd(ld2450_dev_t *dev,
                                 const uint8_t *payload, int payload_len)
{
    esp_err_t err = enter_config(dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "enter_config failed: %s — forcing end_config",
                 esp_err_to_name(err));
        force_end_config(dev);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t pkt[64];
    int pos = 0;
    pkt[pos++] = 0xFD; pkt[pos++] = 0xFC;
    pkt[pos++] = 0xFB; pkt[pos++] = 0xFA;
    pkt[pos++] = (uint8_t)(payload_len & 0xFF);
    pkt[pos++] = (uint8_t)((payload_len >> 8) & 0xFF);
    memcpy(&pkt[pos], payload, payload_len);
    pos += payload_len;
    pkt[pos++] = 0x04; pkt[pos++] = 0x03;
    pkt[pos++] = 0x02; pkt[pos++] = 0x01;

    uint16_t cmd_status = 0;
    err = send_raw(dev, pkt, pos);
    if (err != ESP_OK) {
        force_end_config(dev);
        return err;
    }
    err = wait_ack(dev, &cmd_status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cmd 0x%02X ACK: err=%s status=0x%04X",
                 payload[0], esp_err_to_name(err), cmd_status);
    }

    end_config(dev);
    return err;
}

void ld2450_unstick(ld2450_dev_t *dev)
{
    if (!dev) return;
    force_end_config(dev);
}

esp_err_t ld2450_read_firmware_version(ld2450_dev_t *dev, char *buf, int buf_len)
{
    if (!dev || !buf || buf_len < 2) return ESP_ERR_INVALID_ARG;
    buf[0] = '\0';

    esp_err_t err = enter_config(dev);
    if (err != ESP_OK) {
        force_end_config(dev);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xA0, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    err = send_raw(dev, cmd, sizeof(cmd));
    if (err != ESP_OK) {
        force_end_config(dev);
        return err;
    }

    /* The version ACK has a variable-length string in the payload. */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
    uint8_t b;
    int state = 0;
    uint8_t ack[64];
    int ack_len = 0;
    bool found = false;

    while (xTaskGetTickCount() < deadline) {
        TickType_t remain = deadline - xTaskGetTickCount();
        if (remain == 0) remain = 1;
        int n = uart_read_bytes(dev->uart_num, &b, 1, remain);
        if (n != 1) continue;

        if (state < 4) {
            const uint8_t hdr[] = {0xFD, 0xFC, 0xFB, 0xFA};
            state = (b == hdr[state]) ? state + 1 : (b == 0xFD ? 1 : 0);
        } else {
            ack[ack_len++] = b;
            if (ack_len >= 4 &&
                ack[ack_len - 4] == 0x04 && ack[ack_len - 3] == 0x03 &&
                ack[ack_len - 2] == 0x02 && ack[ack_len - 1] == 0x01) {
                found = true;
                break;
            }
            if (ack_len >= (int)sizeof(ack)) break;
        }
    }

    end_config(dev);

    if (!found || ack_len < 6) return ESP_ERR_NOT_FOUND;

    /* ack: [len_lo, len_hi, cmd_lo, cmd_hi, ...version bytes..., footer(4)] */
    int ver_start = 4;
    int ver_end = ack_len - 4;
    int ver_len = ver_end - ver_start;
    if (ver_len <= 0) return ESP_ERR_NOT_FOUND;
    if (ver_len >= buf_len) ver_len = buf_len - 1;
    memcpy(buf, &ack[ver_start], ver_len);
    buf[ver_len] = '\0';
    return ESP_OK;
}

esp_err_t ld2450_set_sensitivity(ld2450_dev_t *dev, uint8_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (val > 9) val = 9;
    uint8_t payload[10] = {0x64, 0x00};
    for (int i = 0; i < 8; i++)
        payload[2 + i] = val;
    ESP_LOGI(TAG, "UART%d set sensitivity=%u", (int)dev->uart_num, val);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_set_energy_threshold(ld2450_dev_t *dev, uint16_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    uint8_t payload[4] = {
        0x60, 0x00,
        (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF)
    };
    ESP_LOGI(TAG, "UART%d set energy_threshold=%u", (int)dev->uart_num, val);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_set_speed_filter(ld2450_dev_t *dev, uint16_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    uint8_t payload[4] = {
        0x62, 0x00,
        (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF)
    };
    ESP_LOGI(TAG, "UART%d set speed_filter=%u cm/s", (int)dev->uart_num, val);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_set_hold_time(ld2450_dev_t *dev, uint16_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (val > 60) val = 60;
    uint8_t payload[4] = {
        0x61, 0x00,
        (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF)
    };
    ESP_LOGI(TAG, "UART%d set hold_time=%u s", (int)dev->uart_num, val);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_set_zones(ld2450_dev_t *dev, uint16_t type,
                           const ld2450_zone_t zones[3])
{
    if (!dev || !zones) return ESP_ERR_INVALID_ARG;
    uint8_t payload[28];
    payload[0] = 0xC2;
    payload[1] = 0x00;
    payload[2] = (uint8_t)(type & 0xFF);
    payload[3] = (uint8_t)((type >> 8) & 0xFF);
    for (int z = 0; z < 3; z++) {
        int off = 4 + z * 8;
        uint16_t x1 = (uint16_t)zones[z].x1;
        uint16_t y1 = (uint16_t)zones[z].y1;
        uint16_t x2 = (uint16_t)zones[z].x2;
        uint16_t y2 = (uint16_t)zones[z].y2;
        payload[off + 0] = (uint8_t)(x1 & 0xFF);
        payload[off + 1] = (uint8_t)((x1 >> 8) & 0xFF);
        payload[off + 2] = (uint8_t)(y1 & 0xFF);
        payload[off + 3] = (uint8_t)((y1 >> 8) & 0xFF);
        payload[off + 4] = (uint8_t)(x2 & 0xFF);
        payload[off + 5] = (uint8_t)((x2 >> 8) & 0xFF);
        payload[off + 6] = (uint8_t)(y2 & 0xFF);
        payload[off + 7] = (uint8_t)((y2 >> 8) & 0xFF);
    }
    ESP_LOGI(TAG, "UART%d set zones type=%u", (int)dev->uart_num, type);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_restart(ld2450_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    static const uint8_t payload[] = {0xA3, 0x00};
    ESP_LOGI(TAG, "UART%d restart", (int)dev->uart_num);
    return send_config_cmd(dev, payload, sizeof(payload));
}

esp_err_t ld2450_set_output_rate(ld2450_dev_t *dev, uint8_t fps)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    /* Frame rate: opcode 0x0A, parameter is the desired FPS (1-25 typical).
       Different LD2450 firmware variants use different opcodes; if this doesn't
       work, try 0x0B, 0x01, 0x12, etc. and check logs. */
    uint8_t payload[4] = {
        0x0A, 0x00,
        fps, 0x00
    };
    ESP_LOGI(TAG, "UART%d set output_rate=%u fps", (int)dev->uart_num, fps);
    return send_config_cmd(dev, payload, sizeof(payload));
}

/* ── Frame reading ───────────────────────────────────────────────── */

esp_err_t ld2450_read_frame(ld2450_dev_t *dev, ld2450_target_t targets[3],
                            int timeout_ms)
{
    if (!dev || !targets) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeout_ms < 50) {
        timeout_ms = 50;
    }

    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS((uint32_t)timeout_ms);
    uint8_t b;

    while (true) {
        const TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            return ESP_ERR_TIMEOUT;
        }
        const TickType_t wait = deadline - now;
        int n = uart_read_bytes(dev->uart_num, &b, 1, wait);
        if (n != 1) {
            return ESP_ERR_TIMEOUT;
        }
        if (b != FRAME_HEADER_0) {
            continue;
        }
        n = uart_read_bytes(dev->uart_num, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != FRAME_HEADER_1) {
            continue;
        }
        n = uart_read_bytes(dev->uart_num, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != FRAME_HEADER_2) {
            continue;
        }
        n = uart_read_bytes(dev->uart_num, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != FRAME_HEADER_3) {
            continue;
        }
        break;
    }

    uint8_t payload[FRAME_DATA_LEN];
    int got = uart_read_bytes(dev->uart_num, payload, FRAME_DATA_LEN,
                              pdMS_TO_TICKS(100));
    if (got != FRAME_DATA_LEN) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t footer[2];
    got = uart_read_bytes(dev->uart_num, footer, 2, pdMS_TO_TICKS(50));
    if (got != 2 || footer[0] != FRAME_FOOTER_0 || footer[1] != FRAME_FOOTER_1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (int i = 0; i < TARGET_COUNT; ++i) {
        parse_target(&payload[i * TARGET_BYTES], &targets[i]);
    }
    return ESP_OK;
}
