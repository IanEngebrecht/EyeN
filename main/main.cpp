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

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string_view>

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

namespace
{

const char *TAG = "EyeN";

Display s_display;

constexpr int cmd_buf_size = 128;
constexpr int eye_mode_idx = 0;
constexpr int radar_mode_idx = 1;

std::array<DisplayMode *, 2> s_modes = {
    &eye_mode(),
    &radar_mode(),
};

int s_mode_idx = eye_mode_idx;

/* ── Button ──────────────────────────────────────────────────────── */

int64_t s_btn_last_us;

void button_init()
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << cfg::button::gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    s_btn_last_us = esp_timer_get_time();
}

bool button_pressed()
{
    if (gpio_get_level(static_cast<gpio_num_t>(cfg::button::gpio)) != 0)
        return false;
    int64_t now = esp_timer_get_time();
    if ((now - s_btn_last_us) < static_cast<int64_t>(cfg::button::debounce_ms) * 1000)
        return false;
    s_btn_last_us = now;
    return true;
}

/* ── Azimuth coast (shared across modes) ─────────────────────────── */

bool s_az_have;
float s_az_fixed;
float s_az_rate;
int64_t s_az_fixed_us;
uint32_t s_az_last_frame;

float aim_azimuth_deg(const radar::Gaze &g)
{
    const int64_t now = esp_timer_get_time();
    const float az = g.azimuth_deg;

    if (!s_az_have)
    {
        s_az_fixed = az;
        s_az_rate = 0.0f;
        s_az_fixed_us = now;
        s_az_last_frame = g.frame_id;
        s_az_have = true;
        return az;
    }

    if (g.frame_id != s_az_last_frame)
    {
        const float daz = az - s_az_fixed;
        const float dt = static_cast<float>(now - s_az_fixed_us) * 1e-6f;

        if (std::fabs(daz) >= cfg::gaze::az_deadband_deg)
        {
            if (dt > 0.04f && dt < 0.6f)
            {
                const float inst = daz / dt;
                s_az_rate += (inst - s_az_rate) * 0.45f;
            }
            s_az_fixed = az;
        }
        else
        {
            s_az_rate *= 0.35f;
        }
        s_az_fixed_us = now;
        s_az_last_frame = g.frame_id;
    }

    float age = static_cast<float>(now - s_az_fixed_us) * 1e-6f;
    if (age < 0.0f)
        age = 0.0f;
    if (age > 0.25f)
        age = 0.25f;

    float coast = s_az_rate * age;
    if (coast > cfg::gaze::coast_max_deg)
        coast = cfg::gaze::coast_max_deg;
    else if (coast < -cfg::gaze::coast_max_deg)
        coast = -cfg::gaze::coast_max_deg;

    return s_az_fixed + coast;
}

void reset_azimuth_coast()
{
    s_az_have = false;
    s_az_rate = 0.0f;
}

/* ── Logging ─────────────────────────────────────────────────────── */

void log_frame(const radar::Gaze &g, [[maybe_unused]] const ModeFrame &mf)
{
    char buf[512];
    int pos = 0;
    const int cap = static_cast<int>(sizeof(buf)) - 1;

    uint32_t ms = esp_log_timestamp();
    pos += snprintf(buf + pos, cap - pos, "$FRAME t=%lu", static_cast<unsigned long>(ms));

    for (int s = 0; s < g.slot_count && pos < cap; ++s)
    {
        const radar::SlotInfo *sl = &g.slots[s];
        pos += snprintf(buf + pos, cap - pos, " %s=%d:", sl->name, sl->target_count);
        for (int t = 0; t < Ld2450::target_count && pos < cap; ++t)
        {
            if (t > 0)
                buf[pos++] = '|';
            const Target *tg = &sl->targets[t];
            if (tg->valid)
            {
                pos += snprintf(buf + pos, cap - pos, "%d,%d,%u,%d", static_cast<int>(tg->x_mm),
                                static_cast<int>(tg->y_mm), static_cast<unsigned>(tg->distance_mm),
                                static_cast<int>(tg->speed_cm_s));
            }
            else
            {
                buf[pos++] = '-';
            }
        }
    }
    buf[pos] = '\0';
    ESP_LOGI(TAG, "%s", buf);
}

/* ── UART0 commands ──────────────────────────────────────────────── */

void handle_cmd(const char *line)
{
    std::string_view sv{line};
    constexpr std::string_view prefix = "$SET ";
    if (!sv.starts_with(prefix))
        return;
    sv.remove_prefix(prefix.size());

    auto space = sv.find(' ');
    if (space == std::string_view::npos)
        return;

    std::string_view param = sv.substr(0, space);
    std::string_view val_str = sv.substr(space + 1);

    int value = 0;
    auto [ptr, ec] = std::from_chars(val_str.data(), val_str.data() + val_str.size(), value);
    if (ec != std::errc{})
        return;

    radar::FilterConfig filt;
    radar::get_filter(filt);
    bool is_sw = true;

    if (param == "min_speed")
        filt.min_speed_cm_s = value;
    else if (param == "min_dist")
        filt.min_dist_mm = value;
    else if (param == "max_dist")
        filt.max_dist_mm = value;
    else if (param == "persist")
        filt.persist_frames = value;
    else
        is_sw = false;

    if (is_sw)
    {
        radar::set_filter(filt);
        ESP_LOGI(TAG, "$ACK %.*s %d OK", static_cast<int>(param.size()), param.data(), value);
        return;
    }

    int n = radar::slot_count();
    esp_err_t err = ESP_OK;

    for (int i = 0; i < n; i++)
    {
        Ld2450 *dev = radar::get_dev(i);
        if (!dev)
            continue;

        esp_err_t e = ESP_ERR_NOT_SUPPORTED;
        if (param == "sensitivity")
            e = dev->set_sensitivity(static_cast<uint8_t>(value));
        else if (param == "energy")
            e = dev->set_energy_threshold(static_cast<uint16_t>(value));
        else if (param == "speed_filter")
            e = dev->set_speed_filter(static_cast<uint16_t>(value));
        else if (param == "hold_time")
            e = dev->set_hold_time(static_cast<uint16_t>(value));
        else if (param == "restart")
            e = dev->restart();
        if (e != ESP_OK)
            err = e;
    }

    if (err == ESP_OK)
        ESP_LOGI(TAG, "$ACK %.*s %d OK", static_cast<int>(param.size()), param.data(), value);
    else if (err == ESP_ERR_NOT_SUPPORTED)
        ESP_LOGW(TAG, "$ACK %.*s UNKNOWN_PARAM", static_cast<int>(param.size()), param.data());
    else
        ESP_LOGW(TAG, "$ACK %.*s %d FAIL", static_cast<int>(param.size()), param.data(), value);
}

char s_cmd_buf[cmd_buf_size];
int s_cmd_pos = 0;

void poll_uart0_commands()
{
    uint8_t b;
    while (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1)
    {
        if (b == '\n' || b == '\r')
        {
            if (s_cmd_pos > 0)
            {
                s_cmd_buf[s_cmd_pos] = '\0';
                handle_cmd(s_cmd_buf);
                s_cmd_pos = 0;
            }
        }
        else if (s_cmd_pos < cmd_buf_size - 1)
        {
            s_cmd_buf[s_cmd_pos++] = static_cast<char>(b);
        }
    }
}

void init_uart0_rx()
{
    const int buf = 512;
    esp_err_t err = uart_driver_install(UART_NUM_0, buf, 0, 0, nullptr, 0);
    if (err == ESP_ERR_INVALID_STATE)
    {
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "uart0 driver install: %s", esp_err_to_name(err));
    }
}

/* ── Performance instrumentation ─────────────────────────────────── */

constexpr int64_t perf_interval_us = 5'000'000;

int64_t s_perf_start_us;
uint32_t s_perf_frame_count;
int64_t s_perf_frame_min_us;
int64_t s_perf_frame_max_us;
int64_t s_perf_frame_sum_us;

void perf_reset(int64_t now)
{
    s_perf_start_us = now;
    s_perf_frame_count = 0;
    s_perf_frame_min_us = INT64_MAX;
    s_perf_frame_max_us = 0;
    s_perf_frame_sum_us = 0;
}

/* ── Mode switching ──────────────────────────────────────────────── */

void switch_mode(int new_idx)
{
    esp_lcd_panel_handle_t left = s_display.left();
    esp_lcd_panel_handle_t right = s_display.right();

    s_modes[s_mode_idx]->leave();
    s_mode_idx = new_idx;
    s_modes[s_mode_idx]->enter(left, right);
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_mode_idx]->name());
    perf_reset(esp_timer_get_time());
}

bool s_pot_at_limit = false;

void poll_pot_mode_switch(bool &last_human)
{
    float frac = radar::get_pot_frac();
    bool near_limit = frac <= cfg::pot::limit_enter || frac >= (1.0f - cfg::pot::limit_enter);
    bool clear_limit = frac > cfg::pot::limit_exit && frac < (1.0f - cfg::pot::limit_exit);

    if (!s_pot_at_limit && near_limit)
    {
        s_pot_at_limit = true;
        if (s_mode_idx != radar_mode_idx)
        {
            switch_mode(radar_mode_idx);
            reset_azimuth_coast();
            last_human = false;
        }
    }
    else if (s_pot_at_limit && clear_limit)
    {
        s_pot_at_limit = false;
        if (s_mode_idx != eye_mode_idx)
        {
            switch_mode(eye_mode_idx);
            reset_azimuth_coast();
            last_human = false;
        }
    }
}

/* ── Build ModeFrame from radar::Gaze ────────────────────────────── */

void build_mode_frame(const radar::Gaze &g, float az_deg, ModeFrame &mf)
{
    mf.human = g.human;
    mf.azimuth_deg = az_deg;
    mf.elevation_norm = g.elevation_norm;
    mf.frame_id = g.frame_id;
    mf.target_count = g.total_targets;

    mf.targets = {};
    if (g.slot_count > 0)
    {
        for (int i = 0; i < Ld2450::target_count; ++i)
            mf.targets[i] = g.slots[0].targets[i];
    }

    if (g.human)
    {
        mf.primary = g.primary;
        mf.primary_idx = 0;
        for (int i = 0; i < Ld2450::target_count; ++i)
        {
            if (mf.targets[i].valid && mf.targets[i].x_mm == g.primary.x_mm &&
                mf.targets[i].y_mm == g.primary.y_mm)
            {
                mf.primary_idx = i;
                break;
            }
        }
    }
    else
    {
        mf.primary = {};
        mf.primary_idx = -1;
    }
}

} // anonymous namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EyeN starting");

    init_uart0_rx();
    button_init();
    ESP_ERROR_CHECK(s_display.init());
    ESP_ERROR_CHECK(radar::init());

    esp_lcd_panel_handle_t left = s_display.left();
    esp_lcd_panel_handle_t right = s_display.right();
    s_modes[s_mode_idx]->enter(left, right);
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_mode_idx]->name());

    bool last_human = false;
    int log_skip = 0;
    uint32_t last_frame_id = 0;
    uint32_t missed_total = 0;

    perf_reset(esp_timer_get_time());

    while (true)
    {
        poll_uart0_commands();

        if (button_pressed())
        {
            int next = (s_mode_idx + 1) % static_cast<int>(s_modes.size());
            switch_mode(next);
            reset_azimuth_coast();
            last_human = false;
        }

        radar::Gaze gaze;
        esp_err_t err = radar::update(gaze);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "radar update: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        poll_pot_mode_switch(last_human);

        float az_deg = 0.0f;

        if (last_frame_id > 0 && gaze.frame_id > last_frame_id + 1)
        {
            uint32_t skipped = gaze.frame_id - last_frame_id - 1;
            missed_total += skipped;
            ESP_LOGW(TAG, "$SKIP frames=%lu total=%lu", static_cast<unsigned long>(skipped),
                     static_cast<unsigned long>(missed_total));
        }
        last_frame_id = gaze.frame_id;

        if (gaze.human)
        {
            az_deg = aim_azimuth_deg(gaze);
        }
        else
        {
            if (last_human)
            {
                ESP_LOGI(TAG, "$LOST t=%lu", static_cast<unsigned long>(esp_log_timestamp()));
                log_skip = 0;
            }
            reset_azimuth_coast();
        }
        last_human = gaze.human;

        ModeFrame mf;
        build_mode_frame(gaze, az_deg, mf);

        const int64_t t0 = esp_timer_get_time();
        s_modes[s_mode_idx]->render(left, right, mf);
        const int64_t t1 = esp_timer_get_time();

        const int64_t frame_us = t1 - t0;
        s_perf_frame_count++;
        s_perf_frame_sum_us += frame_us;
        s_perf_frame_min_us = std::min(s_perf_frame_min_us, frame_us);
        s_perf_frame_max_us = std::max(s_perf_frame_max_us, frame_us);

        const int64_t elapsed = t1 - s_perf_start_us;
        if (elapsed >= perf_interval_us && s_perf_frame_count > 0)
        {
            const float fps =
                static_cast<float>(s_perf_frame_count) / (static_cast<float>(elapsed) * 1e-6f);
            const int64_t avg_us = s_perf_frame_sum_us / static_cast<int64_t>(s_perf_frame_count);
            ESP_LOGI(TAG, "$PERF mode=%s fps=%.1f frame_us=%lld/%lld/%lld(min/avg/max) frames=%lu",
                     s_modes[s_mode_idx]->name(), static_cast<double>(fps),
                     static_cast<long long>(s_perf_frame_min_us), static_cast<long long>(avg_us),
                     static_cast<long long>(s_perf_frame_max_us),
                     static_cast<unsigned long>(s_perf_frame_count));
            perf_reset(t1);
        }

        if (++log_skip >= 10)
        {
            log_skip = 0;
            log_frame(gaze, mf);
        }
    }
}
