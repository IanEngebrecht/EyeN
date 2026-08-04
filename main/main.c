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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "display.h"
#include "ld2450.h"
#include "mode.h"
#include "radar_stack.h"

static const char *TAG = "EyeN";

#define CMD_BUF_SIZE 128

/* ── Mode management ─────────────────────────────────────────────── */

static const display_mode_t *s_modes[] = {
    &mode_eye,
    &mode_radar,
};
#define MODE_COUNT (sizeof(s_modes) / sizeof(s_modes[0]))
#define EYE_MODE_IDX   0
#define RADAR_MODE_IDX 1

static int s_mode_idx = EYE_MODE_IDX;

/* ── Button ──────────────────────────────────────────────────────── */

static int64_t s_btn_last_us;

static void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CFG_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    s_btn_last_us = esp_timer_get_time();
}

static bool button_pressed(void)
{
    if (gpio_get_level(CFG_BUTTON_GPIO) != 0) return false;
    int64_t now = esp_timer_get_time();
    if ((now - s_btn_last_us) < (int64_t)CFG_BUTTON_DEBOUNCE_MS * 1000)
        return false;
    s_btn_last_us = now;
    return true;
}

/* ── Azimuth coast (shared across modes) ─────────────────────────── */

static bool     s_az_have;
static float    s_az_fixed;
static float    s_az_rate;
static int64_t  s_az_fixed_us;
static uint32_t s_az_last_frame;

static float aim_azimuth_deg(const radar_gaze_t *g)
{
    const int64_t now = esp_timer_get_time();
    const float az = g->azimuth_deg;

    if (!s_az_have) {
        s_az_fixed = az;
        s_az_rate = 0.0f;
        s_az_fixed_us = now;
        s_az_last_frame = g->frame_id;
        s_az_have = true;
        return az;
    }

    if (g->frame_id != s_az_last_frame) {
        const float daz = az - s_az_fixed;
        const float dt = (float)(now - s_az_fixed_us) * 1e-6f;

        if (fabsf(daz) >= CFG_GAZE_AZ_DEADBAND_DEG) {
            if (dt > 0.04f && dt < 0.6f) {
                const float inst = daz / dt;
                s_az_rate += (inst - s_az_rate) * 0.45f;
            }
            s_az_fixed = az;
        } else {
            s_az_rate *= 0.35f;
        }
        s_az_fixed_us = now;
        s_az_last_frame = g->frame_id;
    }

    float age = (float)(now - s_az_fixed_us) * 1e-6f;
    if (age < 0.0f) age = 0.0f;
    if (age > 0.25f) age = 0.25f;

    float coast = s_az_rate * age;
    if (coast > CFG_GAZE_COAST_MAX_DEG) coast = CFG_GAZE_COAST_MAX_DEG;
    else if (coast < -CFG_GAZE_COAST_MAX_DEG) coast = -CFG_GAZE_COAST_MAX_DEG;

    return s_az_fixed + coast;
}

static void reset_azimuth_coast(void)
{
    s_az_have = false;
    s_az_rate = 0.0f;
}

/* ── Logging ─────────────────────────────────────────────────────── */

static void log_frame(const radar_gaze_t *g, const mode_frame_t *mf)
{
    char buf[512];
    int pos = 0;
    const int cap = (int)sizeof(buf) - 1;

    uint32_t ms = esp_log_timestamp();
    pos += snprintf(buf + pos, cap - pos, "$FRAME t=%lu",
                    (unsigned long)ms);

    for (int s = 0; s < g->slot_count && pos < cap; ++s) {
        const radar_slot_info_t *sl = &g->slots[s];
        pos += snprintf(buf + pos, cap - pos, " %s=%d:", sl->name, sl->target_count);
        for (int t = 0; t < RADAR_TARGETS_PER_SLOT && pos < cap; ++t) {
            if (t > 0) buf[pos++] = '|';
            const ld2450_target_t *tg = &sl->targets[t];
            if (tg->valid) {
                pos += snprintf(buf + pos, cap - pos, "%d,%d,%u,%d",
                                (int)tg->x_mm, (int)tg->y_mm,
                                (unsigned)tg->distance_mm,
                                (int)tg->speed_cm_s);
            } else {
                buf[pos++] = '-';
            }
        }
    }
    (void)mf;
    buf[pos] = '\0';
    ESP_LOGI(TAG, "%s", buf);
}

/* ── UART0 commands ──────────────────────────────────────────────── */

static void handle_cmd(const char *line)
{
    char param[32] = {0};
    int value = 0;

    if (sscanf(line, "$SET %31s %d", param, &value) != 2) return;

    radar_filter_cfg_t filt;
    radar_stack_get_filter(&filt);
    bool is_sw = true;

    if (strcmp(param, "min_speed") == 0) {
        filt.min_speed_cm_s = value;
    } else if (strcmp(param, "min_dist") == 0) {
        filt.min_dist_mm = value;
    } else if (strcmp(param, "max_dist") == 0) {
        filt.max_dist_mm = value;
    } else if (strcmp(param, "persist") == 0) {
        filt.persist_frames = value;
    } else {
        is_sw = false;
    }

    if (is_sw) {
        radar_stack_set_filter(&filt);
        ESP_LOGI(TAG, "$ACK %s %d OK", param, value);
        return;
    }

    int n = radar_stack_slot_count();
    esp_err_t err = ESP_OK;

    for (int i = 0; i < n; i++) {
        ld2450_dev_t *dev = radar_stack_get_dev(i);
        if (!dev) continue;

        esp_err_t e = ESP_ERR_NOT_SUPPORTED;
        if (strcmp(param, "sensitivity") == 0) {
            e = ld2450_set_sensitivity(dev, (uint8_t)value);
        } else if (strcmp(param, "energy") == 0) {
            e = ld2450_set_energy_threshold(dev, (uint16_t)value);
        } else if (strcmp(param, "speed_filter") == 0) {
            e = ld2450_set_speed_filter(dev, (uint16_t)value);
        } else if (strcmp(param, "hold_time") == 0) {
            e = ld2450_set_hold_time(dev, (uint16_t)value);
        } else if (strcmp(param, "restart") == 0) {
            e = ld2450_restart(dev);
        }
        if (e != ESP_OK) err = e;
    }

    if (err == ESP_OK)
        ESP_LOGI(TAG, "$ACK %s %d OK", param, value);
    else if (err == ESP_ERR_NOT_SUPPORTED)
        ESP_LOGW(TAG, "$ACK %s UNKNOWN_PARAM", param);
    else
        ESP_LOGW(TAG, "$ACK %s %d FAIL", param, value);
}

static char s_cmd_buf[CMD_BUF_SIZE];
static int  s_cmd_pos = 0;

static void poll_uart0_commands(void)
{
    uint8_t b;
    while (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1) {
        if (b == '\n' || b == '\r') {
            if (s_cmd_pos > 0) {
                s_cmd_buf[s_cmd_pos] = '\0';
                handle_cmd(s_cmd_buf);
                s_cmd_pos = 0;
            }
        } else if (s_cmd_pos < CMD_BUF_SIZE - 1) {
            s_cmd_buf[s_cmd_pos++] = (char)b;
        }
    }
}

static void init_uart0_rx(void)
{
    const int buf = 512;
    esp_err_t err = uart_driver_install(UART_NUM_0, buf, 0, 0, NULL, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        /* already installed */
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart0 driver install: %s", esp_err_to_name(err));
    }
}

/* ── Mode switching ──────────────────────────────────────────────── */

static void switch_mode(int new_idx)
{
    esp_lcd_panel_handle_t left  = display_get_panel_left();
    esp_lcd_panel_handle_t right = display_get_panel_right();

    s_modes[s_mode_idx]->leave();
    s_mode_idx = new_idx;
    s_modes[s_mode_idx]->enter(left, right);
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_mode_idx]->name);
}

/* ── Potentiometer-as-mode-switch (stand-in until a button is wired) ─── */

static bool s_pot_at_limit = false;

/** Dial the pot to either mechanical end -> radar mode; dial back toward
    center -> eye mode. Hysteresis prevents chatter right at the threshold. */
static void poll_pot_mode_switch(bool *last_human)
{
    float frac = radar_stack_get_pot_frac();
    bool near_limit  = frac <= CFG_POT_LIMIT_ENTER || frac >= (1.0f - CFG_POT_LIMIT_ENTER);
    bool clear_limit = frac > CFG_POT_LIMIT_EXIT && frac < (1.0f - CFG_POT_LIMIT_EXIT);

    if (!s_pot_at_limit && near_limit) {
        s_pot_at_limit = true;
        if (s_mode_idx != RADAR_MODE_IDX) {
            switch_mode(RADAR_MODE_IDX);
            reset_azimuth_coast();
            *last_human = false;
        }
    } else if (s_pot_at_limit && clear_limit) {
        s_pot_at_limit = false;
        if (s_mode_idx != EYE_MODE_IDX) {
            switch_mode(EYE_MODE_IDX);
            reset_azimuth_coast();
            *last_human = false;
        }
    }
}

/* ── Build mode_frame_t from radar_gaze_t ────────────────────────── */

static void build_mode_frame(const radar_gaze_t *g, float az_deg, mode_frame_t *mf)
{
    mf->human = g->human;
    mf->azimuth_deg = az_deg;
    mf->elevation_norm = g->elevation_norm;
    mf->frame_id = g->frame_id;
    mf->target_count = g->total_targets;

    /* Copy targets from first slot */
    memset(mf->targets, 0, sizeof(mf->targets));
    if (g->slot_count > 0) {
        for (int i = 0; i < MODE_MAX_TARGETS && i < RADAR_TARGETS_PER_SLOT; ++i)
            mf->targets[i] = g->slots[0].targets[i];
    }

    if (g->human) {
        mf->primary = g->primary;
        /* Find primary index */
        mf->primary_idx = 0;
        for (int i = 0; i < MODE_MAX_TARGETS; ++i) {
            if (mf->targets[i].valid &&
                mf->targets[i].x_mm == g->primary.x_mm &&
                mf->targets[i].y_mm == g->primary.y_mm) {
                mf->primary_idx = i;
                break;
            }
        }
    } else {
        memset(&mf->primary, 0, sizeof(mf->primary));
        mf->primary_idx = -1;
    }
}

/* ── Main ────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "EyeN starting");

    init_uart0_rx();
    button_init();
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(radar_stack_init());

    esp_lcd_panel_handle_t left  = display_get_panel_left();
    esp_lcd_panel_handle_t right = display_get_panel_right();
    s_modes[s_mode_idx]->enter(left, right);
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_mode_idx]->name);

    bool last_human = false;
    int log_skip = 0;
    uint32_t last_frame_id = 0;
    uint32_t missed_total = 0;

    while (true) {
        poll_uart0_commands();

        /* Button cycles modes */
        if (button_pressed()) {
            int next = (s_mode_idx + 1) % MODE_COUNT;
            switch_mode(next);
            reset_azimuth_coast();
            last_human = false;
        }

        radar_gaze_t gaze;
        esp_err_t err = radar_stack_update(&gaze);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "radar_stack_update: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        poll_pot_mode_switch(&last_human);

        float az_deg = 0.0f;

        /* Detect missed frames (frame_id gap > 1) */
        if (last_frame_id > 0 && gaze.frame_id > last_frame_id + 1) {
            uint32_t skipped = gaze.frame_id - last_frame_id - 1;
            missed_total += skipped;
            ESP_LOGW(TAG, "$SKIP frames=%lu total=%lu",
                     (unsigned long)skipped, (unsigned long)missed_total);
        }
        last_frame_id = gaze.frame_id;

        if (gaze.human) {
            az_deg = aim_azimuth_deg(&gaze);
        } else {
            if (last_human) {
                ESP_LOGI(TAG, "$LOST t=%lu", (unsigned long)esp_log_timestamp());
                log_skip = 0;
            }
            reset_azimuth_coast();
        }
        last_human = gaze.human;

        mode_frame_t mf;
        build_mode_frame(&gaze, az_deg, &mf);

        s_modes[s_mode_idx]->render(left, right, &mf);

        if (++log_skip >= 10) {
            log_skip = 0;
            log_frame(&gaze, &mf);
        }
    }
}
