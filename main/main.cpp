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
#include <atomic>
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
#include "mode.h"
#include "radar_stack.h"
#include "rtos/queue.h"
#include "rtos/task.h"

namespace
{

const char *TAG = "EyeN";

/* ── Static FreeRTOS objects (all storage pre-allocated) ─────────── */

rtos::Task<4096> s_radar_task;
rtos::Task<4096> s_render_task;

rtos::Queue<ModeFrame, radar::frame_queue_depth> s_frame_q;
rtos::Queue<radar::DevCommand, radar::cmd_queue_depth> s_cmd_q;

struct ModeSwitch
{
    int mode_idx;
};
rtos::Queue<ModeSwitch, 1> s_mode_q;

std::atomic<float> s_pot_frac{0.5f};

radar::RunCtx s_radar_ctx;

/* ── Constants ──────────────────────────────────────────────────── */

constexpr int cmd_buf_size = 128;
constexpr int eye_mode_idx = 0;
constexpr int radar_mode_idx = 1;

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

/* ── UART0 commands ──────────────────────────────────────────────── */

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

    using K = radar::DevCommand::Kind;
    radar::DevCommand cmd{};
    cmd.value = value;
    bool known = true;

    if (param == "min_speed")
        cmd.kind = K::filter_min_speed;
    else if (param == "min_dist")
        cmd.kind = K::filter_min_dist;
    else if (param == "max_dist")
        cmd.kind = K::filter_max_dist;
    else if (param == "persist")
        cmd.kind = K::filter_persist;
    else if (param == "sensitivity")
        cmd.kind = K::hw_sensitivity;
    else if (param == "energy")
        cmd.kind = K::hw_energy;
    else if (param == "speed_filter")
        cmd.kind = K::hw_speed_filter;
    else if (param == "hold_time")
        cmd.kind = K::hw_hold_time;
    else if (param == "restart")
        cmd.kind = K::hw_restart;
    else
        known = false;

    if (!known)
    {
        ESP_LOGW(TAG, "$ACK %.*s UNKNOWN_PARAM", static_cast<int>(param.size()), param.data());
        return;
    }

    if (s_cmd_q.send(cmd, pdMS_TO_TICKS(50)))
        ESP_LOGI(TAG, "$ACK %.*s %d OK", static_cast<int>(param.size()), param.data(), value);
    else
        ESP_LOGW(TAG, "$ACK %.*s %d QUEUE_FULL", static_cast<int>(param.size()), param.data(),
                 value);
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

/* ── Render task ─────────────────────────────────────────────────── */

Display s_display;

auto s_modes = std::array{&eye_mode(), &radar_mode()};

int s_render_mode_idx = eye_mode_idx;

bool s_az_have;
float s_az_fixed;
float s_az_rate;
int64_t s_az_fixed_us;
uint32_t s_az_last_frame;

/* ── Performance instrumentation ────────────────────────────────── */

constexpr int64_t perf_interval_us = 5'000'000; // 5 seconds

int64_t s_perf_start_us;
uint32_t s_perf_frame_count;
int64_t s_perf_frame_min_us;
int64_t s_perf_frame_max_us;
int64_t s_perf_frame_sum_us;

uint32_t s_perf_radar_last_frame_id;
int64_t s_perf_radar_last_us;
float s_perf_radar_fps;

void perf_reset(int64_t now)
{
    s_perf_start_us = now;
    s_perf_frame_count = 0;
    s_perf_frame_min_us = INT64_MAX;
    s_perf_frame_max_us = 0;
    s_perf_frame_sum_us = 0;
}

float aim_azimuth_deg(float az, uint32_t frame_id)
{
    const int64_t now = esp_timer_get_time();

    if (!s_az_have)
    {
        s_az_fixed = az;
        s_az_rate = 0.0f;
        s_az_fixed_us = now;
        s_az_last_frame = frame_id;
        s_az_have = true;
        return az;
    }

    if (frame_id != s_az_last_frame)
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
        s_az_last_frame = frame_id;
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

void switch_render_mode(int new_idx)
{
    esp_lcd_panel_handle_t left = s_display.left();
    esp_lcd_panel_handle_t right = s_display.right();

    s_modes[s_render_mode_idx]->leave();
    s_render_mode_idx = new_idx;
    s_modes[s_render_mode_idx]->enter(left, right, s_display.scanline_buf());
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_render_mode_idx]->name());
    reset_azimuth_coast();
}

[[noreturn]] void render_loop(void *)
{
    esp_lcd_panel_handle_t left = s_display.left();
    esp_lcd_panel_handle_t right = s_display.right();

    s_modes[s_render_mode_idx]->enter(left, right, s_display.scanline_buf());
    ESP_LOGI(TAG, "$MODE %s", s_modes[s_render_mode_idx]->name());

    ModeFrame mf{};
    bool have_frame = false;

    perf_reset(esp_timer_get_time());
    s_perf_radar_last_frame_id = 0;
    s_perf_radar_last_us = 0;
    s_perf_radar_fps = 0.0f;

    while (true)
    {
        ModeSwitch ms;
        if (s_mode_q.receive(ms, 0))
            switch_render_mode(ms.mode_idx);

        ModeFrame new_mf;
        if (s_frame_q.receive(new_mf, pdMS_TO_TICKS(10)))
        {
            /* Track radar update rate via frame_id deltas */
            const int64_t rx_now = esp_timer_get_time();
            if (s_perf_radar_last_us != 0 && new_mf.frame_id != s_perf_radar_last_frame_id)
            {
                const uint32_t id_delta = new_mf.frame_id - s_perf_radar_last_frame_id;
                const float dt = static_cast<float>(rx_now - s_perf_radar_last_us) * 1e-6f;
                if (dt > 0.0f)
                {
                    const float instant = static_cast<float>(id_delta) / dt;
                    /* Exponential moving average (alpha ~0.3) */
                    s_perf_radar_fps += (instant - s_perf_radar_fps) * 0.3f;
                }
            }
            s_perf_radar_last_frame_id = new_mf.frame_id;
            s_perf_radar_last_us = rx_now;

            mf = new_mf;
            have_frame = true;
        }

        if (!have_frame)
            continue;

        if (mf.human)
            mf.azimuth_deg = aim_azimuth_deg(mf.azimuth_deg, mf.frame_id);

        /* Measure render() call duration */
        const int64_t t0 = esp_timer_get_time();
        s_modes[s_render_mode_idx]->render(left, right, mf);
        const int64_t t1 = esp_timer_get_time();

        const int64_t frame_us = t1 - t0;
        s_perf_frame_count++;
        s_perf_frame_sum_us += frame_us;
        s_perf_frame_min_us = std::min(s_perf_frame_min_us, frame_us);
        s_perf_frame_max_us = std::max(s_perf_frame_max_us, frame_us);

        /* Periodic perf log */
        const int64_t elapsed = t1 - s_perf_start_us;
        if (elapsed >= perf_interval_us && s_perf_frame_count > 0)
        {
            const float period_s = static_cast<float>(elapsed) * 1e-6f;
            const float render_fps = static_cast<float>(s_perf_frame_count) / period_s;
            const int64_t avg_us = s_perf_frame_sum_us / static_cast<int64_t>(s_perf_frame_count);
            ESP_LOGI(TAG,
                     "$PERF render_fps=%.1f frame_us=%lld/%lld/%lld(min/avg/max) frames=%lu "
                     "radar_fps=%.1f",
                     static_cast<double>(render_fps), static_cast<long long>(s_perf_frame_min_us),
                     static_cast<long long>(avg_us), static_cast<long long>(s_perf_frame_max_us),
                     static_cast<unsigned long>(s_perf_frame_count),
                     static_cast<double>(s_perf_radar_fps));
            perf_reset(t1);
        }
    }
}

/* ── Mode switching (control plane) ──────────────────────────────── */

int s_ctrl_mode_idx = eye_mode_idx;
void request_mode_switch(int new_idx)
{
    if (new_idx == s_ctrl_mode_idx)
        return;
    s_ctrl_mode_idx = new_idx;
    ModeSwitch ms{new_idx};
    s_mode_q.send(ms, pdMS_TO_TICKS(50));
}

} // anonymous namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EyeN starting");

    init_uart0_rx();
    button_init();
    ESP_ERROR_CHECK(s_display.init());
    ESP_ERROR_CHECK(radar::init());

    s_frame_q.create();
    s_cmd_q.create();
    s_mode_q.create();

    s_radar_ctx = {
        .frame_q = &s_frame_q,
        .cmd_q = &s_cmd_q,
        .pot_frac = &s_pot_frac,
    };

    s_render_task.create("render", 5, render_loop, nullptr, 0);
    s_radar_task.create("radar", 6, radar::run, &s_radar_ctx, 1);

    const uint32_t init_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "tasks spawned, heap=%lu", static_cast<unsigned long>(init_heap));

    TickType_t next_health = xTaskGetTickCount() + pdMS_TO_TICKS(30'000);

    while (true)
    {
        poll_uart0_commands();

        if (button_pressed())
        {
            int next = (s_ctrl_mode_idx + 1) % static_cast<int>(s_modes.size());
            request_mode_switch(next);
        }

        if ((int32_t)(xTaskGetTickCount() - next_health) >= 0)
        {
            uint32_t heap = esp_get_free_heap_size();
            int32_t delta = static_cast<int32_t>(heap) - static_cast<int32_t>(init_heap);
            UBaseType_t radar_stk = uxTaskGetStackHighWaterMark(s_radar_task.handle());
            UBaseType_t render_stk = uxTaskGetStackHighWaterMark(s_render_task.handle());
            ESP_LOGI(TAG, "$HEALTH heap=%lu delta=%ld radar_stk=%lu render_stk=%lu",
                     static_cast<unsigned long>(heap), static_cast<long>(delta),
                     static_cast<unsigned long>(radar_stk), static_cast<unsigned long>(render_stk));
            next_health = xTaskGetTickCount() + pdMS_TO_TICKS(60'000);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
