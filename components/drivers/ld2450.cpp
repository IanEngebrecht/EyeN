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

#include <algorithm>
#include <cmath>
#include <cstring>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ld2450";

namespace
{

constexpr uint8_t frame_header_0 = 0xAA;
constexpr uint8_t frame_header_1 = 0xFF;
constexpr uint8_t frame_header_2 = 0x03;
constexpr uint8_t frame_header_3 = 0x00;
constexpr uint8_t frame_footer_0 = 0x55;
constexpr uint8_t frame_footer_1 = 0xCC;

constexpr int target_bytes = 8;
constexpr int frame_data_len = target_bytes * Ld2450::target_count;
constexpr int uart_buf_size = 1024;

constexpr int max_abs_x_mm = 6000;
constexpr int max_abs_y_mm = 8000;

constexpr uint8_t cmd_header_0 = 0xFD;
constexpr uint8_t cmd_header_1 = 0xFC;
constexpr uint8_t cmd_header_2 = 0xFB;
constexpr uint8_t cmd_header_3 = 0xFA;

constexpr uint8_t cmd_footer_0 = 0x04;
constexpr uint8_t cmd_footer_1 = 0x03;
constexpr uint8_t cmd_footer_2 = 0x02;
constexpr uint8_t cmd_footer_3 = 0x01;

constexpr int ack_timeout_ms = 500;

int16_t decode_signed_le(const uint8_t *p)
{
    const uint16_t raw = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
    return (raw & 0x8000) ? static_cast<int16_t>(-mag) : mag;
}

uint16_t read_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

void parse_target(const uint8_t *p, Target &t)
{
    bool all_zero = true;
    for (int i = 0; i < target_bytes; ++i)
    {
        if (p[i] != 0)
        {
            all_zero = false;
            break;
        }
    }
    if (all_zero)
    {
        t = Target{};
        return;
    }

    t.x_mm = decode_signed_le(p);
    t.y_mm = decode_signed_le(p + 2);
    t.speed_cm_s = decode_signed_le(p + 4);
    (void)read_u16_le(p + 6);

    const float dist = sqrtf(static_cast<float>(t.x_mm) * static_cast<float>(t.x_mm) +
                             static_cast<float>(t.y_mm) * static_cast<float>(t.y_mm));
    t.distance_mm = static_cast<uint16_t>(dist > 65535.0f ? 65535.0f : dist);

    const int ax = t.x_mm < 0 ? -t.x_mm : t.x_mm;
    const int ay = t.y_mm < 0 ? -t.y_mm : t.y_mm;
    t.valid = (ax <= max_abs_x_mm && ay <= max_abs_y_mm && t.distance_mm > 0);
}

} // anonymous namespace

/* ── Lifecycle ───────────────────────────────────────────────────── */

void Ld2450::release()
{
    if (valid_)
    {
        uart_driver_delete(uart_num_);
        valid_ = false;
    }
}

Ld2450::~Ld2450()
{
    release();
}

Ld2450::Ld2450(Ld2450 &&other) noexcept : uart_num_(other.uart_num_), valid_(other.valid_)
{
    other.valid_ = false;
}

Ld2450 &Ld2450::operator=(Ld2450 &&other) noexcept
{
    if (this != &other)
    {
        release();
        uart_num_ = other.uart_num_;
        valid_ = other.valid_;
        other.valid_ = false;
    }
    return *this;
}

esp_err_t Ld2450::create(const Config &cfg, Ld2450 &out)
{
    out.release();
    out.uart_num_ = cfg.uart_num;

    const uart_config_t uart_cfg = {
        .baud_rate = cfg.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    esp_err_t err = uart_driver_install(cfg.uart_num, uart_buf_size * 2, 0, 0, nullptr, 0);
    if (err != ESP_OK)
        return err;

    err = uart_param_config(cfg.uart_num, &uart_cfg);
    if (err != ESP_OK)
    {
        uart_driver_delete(cfg.uart_num);
        return err;
    }

    const int tx = (cfg.tx_gpio < 0) ? UART_PIN_NO_CHANGE : cfg.tx_gpio;
    const int rx = cfg.rx_gpio;
    err = uart_set_pin(cfg.uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        uart_driver_delete(cfg.uart_num);
        return err;
    }

    out.valid_ = true;
    ESP_LOGI(TAG, "UART%d @ %d RX=%d TX=%d", static_cast<int>(cfg.uart_num), cfg.baud, rx, tx);
    return ESP_OK;
}

/* ── Configuration command infrastructure ─────────────────────────── */

esp_err_t Ld2450::send_raw(const uint8_t *data, int len)
{
    int written = uart_write_bytes(uart_num_, data, len);
    if (written != len)
        return ESP_FAIL;
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t Ld2450::wait_ack(uint16_t *out_status)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ack_timeout_ms);
    uint8_t b;
    int state = 0;
    uint8_t ack_buf[64];
    int ack_len = 0;

    while (xTaskGetTickCount() < deadline)
    {
        TickType_t remain = deadline - xTaskGetTickCount();
        if (remain == 0)
            remain = 1;
        int n = uart_read_bytes(uart_num_, &b, 1, remain);
        if (n != 1)
            continue;

        if (state < 4)
        {
            constexpr uint8_t expect[] = {cmd_header_0, cmd_header_1, cmd_header_2, cmd_header_3};
            state = (b == expect[state]) ? state + 1 : (b == cmd_header_0 ? 1 : 0);
        }
        else
        {
            ack_buf[ack_len++] = b;
            if (ack_len >= 4 && ack_buf[ack_len - 4] == cmd_footer_0 &&
                ack_buf[ack_len - 3] == cmd_footer_1 && ack_buf[ack_len - 2] == cmd_footer_2 &&
                ack_buf[ack_len - 1] == cmd_footer_3)
            {
                uint16_t status = 0;
                if (ack_len >= 6)
                    status = static_cast<uint16_t>(ack_buf[4]) |
                             (static_cast<uint16_t>(ack_buf[5]) << 8);
                if (out_status)
                    *out_status = status;
                return (status == 0) ? ESP_OK : ESP_FAIL;
            }
            if (ack_len >= static_cast<int>(sizeof(ack_buf)))
                return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_ERR_TIMEOUT;
}

void Ld2450::force_end_config()
{
    static constexpr uint8_t cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00,
                                      0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    send_raw(cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(uart_num_);
}

esp_err_t Ld2450::enter_config()
{
    static constexpr uint8_t cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF,
                                      0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    uart_flush_input(uart_num_);
    vTaskDelay(pdMS_TO_TICKS(30));
    uart_flush_input(uart_num_);

    esp_err_t err = send_raw(cmd, sizeof(cmd));
    if (err != ESP_OK)
        return err;
    return wait_ack(nullptr);
}

esp_err_t Ld2450::end_config()
{
    static constexpr uint8_t cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00,
                                      0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    esp_err_t err = send_raw(cmd, sizeof(cmd));
    if (err != ESP_OK)
        return err;
    esp_err_t ack = wait_ack(nullptr);
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(uart_num_);
    return ack;
}

esp_err_t Ld2450::send_config_cmd(const uint8_t *payload, int payload_len)
{
    esp_err_t err = enter_config();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "enter_config failed: %s — forcing end_config", esp_err_to_name(err));
        force_end_config();
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t pkt[64];
    int pos = 0;
    pkt[pos++] = 0xFD;
    pkt[pos++] = 0xFC;
    pkt[pos++] = 0xFB;
    pkt[pos++] = 0xFA;
    pkt[pos++] = static_cast<uint8_t>(payload_len & 0xFF);
    pkt[pos++] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
    std::memcpy(&pkt[pos], payload, payload_len);
    pos += payload_len;
    pkt[pos++] = 0x04;
    pkt[pos++] = 0x03;
    pkt[pos++] = 0x02;
    pkt[pos++] = 0x01;

    uint16_t cmd_status = 0;
    err = send_raw(pkt, pos);
    if (err != ESP_OK)
    {
        force_end_config();
        return err;
    }
    err = wait_ack(&cmd_status);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "cmd 0x%02X ACK: err=%s status=0x%04X", payload[0], esp_err_to_name(err),
                 cmd_status);
    }

    end_config();
    return err;
}

void Ld2450::unstick()
{
    if (!valid_)
        return;
    force_end_config();
}

esp_err_t Ld2450::read_firmware_version(std::span<char> buf)
{
    if (!valid_ || buf.size() < 2)
        return ESP_ERR_INVALID_ARG;
    buf[0] = '\0';

    esp_err_t err = enter_config();
    if (err != ESP_OK)
    {
        force_end_config();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    static constexpr uint8_t cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00,
                                      0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    err = send_raw(cmd, sizeof(cmd));
    if (err != ESP_OK)
    {
        force_end_config();
        return err;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ack_timeout_ms);
    uint8_t b;
    int state = 0;
    uint8_t ack[64];
    int ack_len = 0;
    bool found = false;

    while (xTaskGetTickCount() < deadline)
    {
        TickType_t remain = deadline - xTaskGetTickCount();
        if (remain == 0)
            remain = 1;
        int n = uart_read_bytes(uart_num_, &b, 1, remain);
        if (n != 1)
            continue;

        if (state < 4)
        {
            constexpr uint8_t hdr[] = {0xFD, 0xFC, 0xFB, 0xFA};
            state = (b == hdr[state]) ? state + 1 : (b == 0xFD ? 1 : 0);
        }
        else
        {
            ack[ack_len++] = b;
            if (ack_len >= 4 && ack[ack_len - 4] == 0x04 && ack[ack_len - 3] == 0x03 &&
                ack[ack_len - 2] == 0x02 && ack[ack_len - 1] == 0x01)
            {
                found = true;
                break;
            }
            if (ack_len >= static_cast<int>(sizeof(ack)))
                break;
        }
    }

    end_config();

    if (!found || ack_len < 6)
        return ESP_ERR_NOT_FOUND;

    int ver_start = 4;
    int ver_end = ack_len - 4;
    int ver_len = ver_end - ver_start;
    if (ver_len <= 0)
        return ESP_ERR_NOT_FOUND;
    if (ver_len >= static_cast<int>(buf.size()))
        ver_len = static_cast<int>(buf.size()) - 1;
    std::memcpy(buf.data(), &ack[ver_start], ver_len);
    buf[ver_len] = '\0';
    return ESP_OK;
}

esp_err_t Ld2450::set_sensitivity(uint8_t val)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    if (val > 9)
        val = 9;
    uint8_t payload[10] = {0x64, 0x00};
    for (int i = 0; i < 8; i++)
        payload[2 + i] = val;
    ESP_LOGI(TAG, "UART%d set sensitivity=%u", static_cast<int>(uart_num_), val);
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::set_energy_threshold(uint16_t val)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    uint8_t payload[4] = {0x60, 0x00, static_cast<uint8_t>(val & 0xFF),
                          static_cast<uint8_t>((val >> 8) & 0xFF)};
    ESP_LOGI(TAG, "UART%d set energy_threshold=%u", static_cast<int>(uart_num_), val);
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::set_speed_filter(uint16_t val)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    uint8_t payload[4] = {0x62, 0x00, static_cast<uint8_t>(val & 0xFF),
                          static_cast<uint8_t>((val >> 8) & 0xFF)};
    ESP_LOGI(TAG, "UART%d set speed_filter=%u cm/s", static_cast<int>(uart_num_), val);
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::set_hold_time(uint16_t val)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    if (val > 60)
        val = 60;
    uint8_t payload[4] = {0x61, 0x00, static_cast<uint8_t>(val & 0xFF),
                          static_cast<uint8_t>((val >> 8) & 0xFF)};
    ESP_LOGI(TAG, "UART%d set hold_time=%u s", static_cast<int>(uart_num_), val);
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::set_zones(uint16_t type, const std::array<Zone, 3> &zones)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    uint8_t payload[28];
    payload[0] = 0xC2;
    payload[1] = 0x00;
    payload[2] = static_cast<uint8_t>(type & 0xFF);
    payload[3] = static_cast<uint8_t>((type >> 8) & 0xFF);
    for (int z = 0; z < 3; z++)
    {
        int off = 4 + z * 8;
        auto x1 = static_cast<uint16_t>(zones[z].x1);
        auto y1 = static_cast<uint16_t>(zones[z].y1);
        auto x2 = static_cast<uint16_t>(zones[z].x2);
        auto y2 = static_cast<uint16_t>(zones[z].y2);
        payload[off + 0] = static_cast<uint8_t>(x1 & 0xFF);
        payload[off + 1] = static_cast<uint8_t>((x1 >> 8) & 0xFF);
        payload[off + 2] = static_cast<uint8_t>(y1 & 0xFF);
        payload[off + 3] = static_cast<uint8_t>((y1 >> 8) & 0xFF);
        payload[off + 4] = static_cast<uint8_t>(x2 & 0xFF);
        payload[off + 5] = static_cast<uint8_t>((x2 >> 8) & 0xFF);
        payload[off + 6] = static_cast<uint8_t>(y2 & 0xFF);
        payload[off + 7] = static_cast<uint8_t>((y2 >> 8) & 0xFF);
    }
    ESP_LOGI(TAG, "UART%d set zones type=%u", static_cast<int>(uart_num_), type);
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::restart()
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    static constexpr uint8_t payload[] = {0xA3, 0x00};
    ESP_LOGI(TAG, "UART%d restart", static_cast<int>(uart_num_));
    return send_config_cmd(payload, sizeof(payload));
}

esp_err_t Ld2450::set_output_rate(uint8_t fps)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    uint8_t payload[4] = {0x0A, 0x00, fps, 0x00};
    ESP_LOGI(TAG, "UART%d set output_rate=%u fps", static_cast<int>(uart_num_), fps);
    return send_config_cmd(payload, sizeof(payload));
}

/* ── Frame reading ───────────────────────────────────────────────── */

esp_err_t Ld2450::read_frame(std::array<Target, target_count> &targets, int timeout_ms)
{
    if (!valid_)
        return ESP_ERR_INVALID_ARG;
    if (timeout_ms < 50)
        timeout_ms = 50;

    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(static_cast<uint32_t>(timeout_ms));
    uint8_t b;

    while (true)
    {
        const TickType_t now = xTaskGetTickCount();
        if (now >= deadline)
            return ESP_ERR_TIMEOUT;
        const TickType_t wait = deadline - now;
        int n = uart_read_bytes(uart_num_, &b, 1, wait);
        if (n != 1)
            return ESP_ERR_TIMEOUT;
        if (b != frame_header_0)
            continue;
        n = uart_read_bytes(uart_num_, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != frame_header_1)
            continue;
        n = uart_read_bytes(uart_num_, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != frame_header_2)
            continue;
        n = uart_read_bytes(uart_num_, &b, 1, pdMS_TO_TICKS(50));
        if (n != 1 || b != frame_header_3)
            continue;
        break;
    }

    uint8_t payload[frame_data_len];
    int got = uart_read_bytes(uart_num_, payload, frame_data_len, pdMS_TO_TICKS(100));
    if (got != frame_data_len)
        return ESP_ERR_TIMEOUT;

    uint8_t footer[2];
    got = uart_read_bytes(uart_num_, footer, 2, pdMS_TO_TICKS(50));
    if (got != 2 || footer[0] != frame_footer_0 || footer[1] != frame_footer_1)
        return ESP_ERR_INVALID_RESPONSE;

    for (int i = 0; i < target_count; ++i)
        parse_target(&payload[i * target_bytes], targets[i]);

    return ESP_OK;
}
