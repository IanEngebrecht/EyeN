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

#include "radar_stack.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <numbers>

#include "ld2450.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

static const char *TAG = "radar_single";

namespace
{

const cfg::SensorConfig sensor_cfg[] = {
    cfg::sensors[0],
};
constexpr int sensor_count = static_cast<int>(sizeof(sensor_cfg) / sizeof(sensor_cfg[0]));

int s_slot = -1;
Ld2450 s_dev;
Target s_targets[Ld2450::target_count]{};
TickType_t s_last_ok_tick;
bool s_has_data;
uint32_t s_frame_id;

adc_oneshot_unit_handle_t s_adc;
float s_pot_filt;
bool s_pot_ok;

int s_focus_idx = -1;
TickType_t s_hold_deadline;
TickType_t s_motion_lock_until;

radar::FilterConfig s_filter = {
    .min_speed_cm_s = cfg::filter::min_speed_cm_s,
    .min_dist_mm = cfg::filter::min_dist_mm,
    .max_dist_mm = cfg::filter::max_dist_mm,
    .persist_frames = cfg::filter::persist_frames,
};

radar::PersistSlot s_persist[Ld2450::target_count]{};

TickType_t random_hold_ticks()
{
    const uint32_t span = static_cast<uint32_t>(cfg::gaze::hold_max_ms - cfg::gaze::hold_min_ms);
    uint32_t ms = static_cast<uint32_t>(cfg::gaze::hold_min_ms);
    if (span > 0)
        ms += esp_random() % (span + 1);
    return pdMS_TO_TICKS(ms);
}

void arm_hold()
{
    s_hold_deadline = xTaskGetTickCount() + random_hold_ticks();
}

bool select_attention_target(std::span<const Target> tgt, Target &out)
{
    int valid[Ld2450::target_count];
    int n = 0;
    for (int i = 0; i < Ld2450::target_count; ++i)
    {
        if (tgt[i].valid)
            valid[n++] = i;
    }

    if (n == 0)
    {
        s_focus_idx = -1;
        return false;
    }

    if (n == 1)
    {
        s_focus_idx = valid[0];
        arm_hold();
        out = tgt[s_focus_idx];
        return true;
    }

    const TickType_t now = xTaskGetTickCount();
    const bool focus_valid =
        s_focus_idx >= 0 && s_focus_idx < Ld2450::target_count && tgt[s_focus_idx].valid;
    const bool focus_moving =
        focus_valid && std::abs(tgt[s_focus_idx].speed_cm_s) >= cfg::gaze::motion_cm_s;

    if (focus_moving)
    {
        arm_hold();
        out = tgt[s_focus_idx];
        return true;
    }

    int mover = -1;
    int mover_spd = cfg::gaze::motion_cm_s - 1;
    for (int k = 0; k < n; ++k)
    {
        int i = valid[k];
        int spd = std::abs(tgt[i].speed_cm_s);
        if (spd >= cfg::gaze::motion_cm_s && spd > mover_spd)
        {
            mover = i;
            mover_spd = spd;
        }
    }

    if (mover >= 0)
    {
        const bool locked = focus_valid && (int32_t)(now - s_motion_lock_until) < 0;

        if (!focus_valid || !locked)
        {
            if (s_focus_idx != mover)
            {
                ESP_LOGI(TAG, "gaze → moving slot %d (|v|=%d cm/s)", mover, mover_spd);
                s_motion_lock_until = now + pdMS_TO_TICKS(cfg::gaze::motion_lock_ms);
            }
            s_focus_idx = mover;
            arm_hold();
            out = tgt[s_focus_idx];
            return true;
        }
        arm_hold();
        out = tgt[s_focus_idx];
        return true;
    }

    if (!focus_valid)
    {
        s_focus_idx = radar::nearest_target_idx(tgt);
        if (s_focus_idx < 0)
            return false;
        arm_hold();
        out = tgt[s_focus_idx];
        return true;
    }

    if ((int32_t)(now - s_hold_deadline) >= 0)
    {
        int pos = 0;
        for (int k = 0; k < n; ++k)
        {
            if (valid[k] == s_focus_idx)
            {
                pos = k;
                break;
            }
        }
        int next = valid[(pos + 1) % n];
        ESP_LOGI(TAG, "gaze rotate slot %d → %d (%d targets)", s_focus_idx, next, n);
        s_focus_idx = next;
        arm_hold();
    }

    out = tgt[s_focus_idx];
    return true;
}

bool slot_is_current()
{
    if (!s_has_data)
        return false;
    TickType_t age = xTaskGetTickCount() - s_last_ok_tick;
    return age < pdMS_TO_TICKS(cfg::frame::stale_ms);
}

esp_err_t pot_init()
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = cfg::pot::adc_unit;
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC unit: %s (pot disabled)", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = cfg::pot::atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, cfg::pot::adc_channel, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC channel: %s (pot disabled)", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = nullptr;
        return err;
    }

    s_pot_ok = true;
    s_pot_filt = 0.5f;
    ESP_LOGI(TAG, "pot on GPIO%d (ADC), mount %d‥%d mm", cfg::pot::gpio, cfg::pot::mount_min_mm,
             cfg::pot::mount_max_mm);
    return ESP_OK;
}

void pot_update()
{
    if (!s_pot_ok || !s_adc)
        return;

    int raw = 0;
    if (adc_oneshot_read(s_adc, cfg::pot::adc_channel, &raw) != ESP_OK)
        return;

    float n = static_cast<float>(raw) / 4095.0f;
    if (n < 0.0f)
        n = 0.0f;
    if (n > 1.0f)
        n = 1.0f;
    s_pot_filt += (n - s_pot_filt) * cfg::pot::smooth;
}

float pot_mount_height_mm()
{
    float frac = s_pot_ok ? s_pot_filt : 0.5f;
    return cfg::pot::mount_min_mm +
           frac * static_cast<float>(cfg::pot::mount_max_mm - cfg::pot::mount_min_mm);
}

float elevation_from_distance(const Target &t)
{
    float y = std::fabs(static_cast<float>(t.y_mm));
    if (y < 1.0f)
        y = 1.0f;

    float dh = static_cast<float>(cfg::pot::person_aim_mm) - pot_mount_height_mm();
    float elev_deg = std::atan2(dh, y) * (180.0f / std::numbers::pi_v<float>);

    float deg = elev_deg;
    if (deg < -cfg::gaze::max_deg)
        deg = -cfg::gaze::max_deg;
    else if (deg > cfg::gaze::max_deg)
        deg = cfg::gaze::max_deg;

    return (deg + cfg::gaze::max_deg) / (2.0f * cfg::gaze::max_deg);
}

/* ── Internal update (was public radar::update) ─────────────────── */

esp_err_t do_update(radar::Gaze &out)
{
    out = radar::Gaze{};

    pot_update();

    if (!s_dev.valid() || s_slot < 0)
        return ESP_ERR_INVALID_STATE;

    const cfg::SensorConfig *sc = &sensor_cfg[s_slot];

    std::array<Target, Ld2450::target_count> tmp{};
    esp_err_t err = s_dev.read_frame(tmp, cfg::frame::timeout_ms);
    if (err == ESP_OK)
    {
        if (sc->inverted)
        {
            for (auto &t : tmp)
                t.x_mm = -t.x_mm;
        }
        radar::filter_targets(tmp, s_persist, s_filter);
        std::memcpy(s_targets, tmp.data(), sizeof(s_targets));
        s_last_ok_tick = xTaskGetTickCount();
        s_has_data = true;
        s_frame_id++;
    }
    else if (err != ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "%s frame: %s", sc->name, esp_err_to_name(err));
    }

    out.frame_id = s_frame_id;

    radar::SlotInfo &info = out.slots[0];
    info.name = sc->name;
    if (slot_is_current())
    {
        std::memcpy(info.targets, s_targets, sizeof(info.targets));
        info.target_count = radar::count_valid(s_targets);
    }
    out.slot_count = 1;
    out.total_targets = info.target_count;

    if (!slot_is_current())
    {
        out.human = false;
        return ESP_OK;
    }

    Target focus;
    if (!select_attention_target(s_targets, focus))
    {
        out.human = false;
        return ESP_OK;
    }

    out.human = true;
    out.primary = focus;
    out.azimuth_deg = radar::target_azimuth_deg(focus);
    out.elevation_norm = elevation_from_distance(focus);
    out.see_mask = 0x01;
    return ESP_OK;
}

/* ── Build ModeFrame from Gaze ──────────────────────────────────── */

void build_mode_frame(const radar::Gaze &g, ModeFrame &mf)
{
    mf.human = g.human;
    mf.azimuth_deg = g.azimuth_deg;
    mf.elevation_norm = g.elevation_norm;
    mf.frame_id = g.frame_id;
    mf.target_count = g.total_targets;

    mf.targets = {};
    if (g.slot_count > 0)
    {
        for (int i = 0; i < mode_frame_max_targets; ++i)
            mf.targets[i] = g.slots[0].targets[i];
    }

    if (g.human)
    {
        mf.primary = g.primary;
        mf.primary_idx = 0;
        for (int i = 0; i < mode_frame_max_targets; ++i)
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

/* ── Frame logging ──────────────────────────────────────────────── */

void log_frame(const radar::Gaze &g)
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
        for (int t = 0; t < mode_frame_max_targets && pos < cap; ++t)
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

/* ── DevCommand handling ────────────────────────────────────────── */

void handle_dev_command(const radar::DevCommand &cmd)
{
    using K = radar::DevCommand::Kind;

    switch (cmd.kind)
    {
    case K::filter_min_speed:
        s_filter.min_speed_cm_s = cmd.value;
        break;
    case K::filter_min_dist:
        s_filter.min_dist_mm = cmd.value;
        break;
    case K::filter_max_dist:
        s_filter.max_dist_mm = cmd.value;
        break;
    case K::filter_persist:
        s_filter.persist_frames = cmd.value;
        break;
    case K::hw_sensitivity:
        if (s_dev.valid())
            s_dev.set_sensitivity(static_cast<uint8_t>(cmd.value));
        break;
    case K::hw_energy:
        if (s_dev.valid())
            s_dev.set_energy_threshold(static_cast<uint16_t>(cmd.value));
        break;
    case K::hw_speed_filter:
        if (s_dev.valid())
            s_dev.set_speed_filter(static_cast<uint16_t>(cmd.value));
        break;
    case K::hw_hold_time:
        if (s_dev.valid())
            s_dev.set_hold_time(static_cast<uint16_t>(cmd.value));
        break;
    case K::hw_restart:
        if (s_dev.valid())
            s_dev.restart();
        break;
    }

    ESP_LOGI(TAG, "filter: min_spd=%d min_d=%d max_d=%d persist=%d", s_filter.min_speed_cm_s,
             s_filter.min_dist_mm, s_filter.max_dist_mm, s_filter.persist_frames);
}

} // anonymous namespace

namespace radar
{

esp_err_t init()
{
    s_has_data = false;
    s_slot = -1;
    s_focus_idx = -1;
    s_hold_deadline = 0;
    s_motion_lock_until = 0;
    memset(s_targets, 0, sizeof(s_targets));
    memset(s_persist, 0, sizeof(s_persist));

    for (int i = 0; i < sensor_count; ++i)
    {
        if (sensor_cfg[i].enabled)
        {
            s_slot = i;
            break;
        }
    }
    if (s_slot < 0)
    {
        ESP_LOGE(TAG, "no enabled sensor in cfg::sensors");
        return ESP_ERR_INVALID_STATE;
    }

    const cfg::SensorConfig *sc = &sensor_cfg[s_slot];
    const Ld2450::Config uart_cfg = {
        .uart_num = sc->uart_num,
        .tx_gpio = sc->tx_gpio,
        .rx_gpio = sc->rx_gpio,
        .baud = cfg::radar::baud,
    };
    esp_err_t err = Ld2450::create(uart_cfg, s_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "create %s failed: %s", sc->name, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "sensor %-6s  pitch=%+3d°  inv=%d  uart=%d  rx=%d  tx=%d", sc->name,
             sc->pitch_deg, sc->inverted, static_cast<int>(sc->uart_num), sc->rx_gpio, sc->tx_gpio);

    if (sc->tx_gpio >= 0)
    {
        s_dev.unstick();
        char ver[32];
        if (s_dev.read_firmware_version(ver) == ESP_OK)
            ESP_LOGI(TAG, "  %s firmware: %s", sc->name, ver);
        else
            ESP_LOGW(TAG, "  %s firmware: could not read (TX wired?)", sc->name);
    }

    pot_init();
    return ESP_OK;
}

[[noreturn]] void run(void *ctx)
{
    auto *rc = static_cast<RunCtx *>(ctx);

    bool last_human = false;
    int log_skip = 0;
    uint32_t last_frame_id = 0;
    uint32_t missed_total = 0;

    while (true)
    {
        DevCommand cmd;
        while (rc->cmd_q->receive(cmd, 0))
            handle_dev_command(cmd);

        Gaze gaze;
        esp_err_t err = do_update(gaze);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "radar update: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        rc->pot_frac->store(s_pot_ok ? s_pot_filt : 0.5f, std::memory_order_relaxed);

        if (last_frame_id > 0 && gaze.frame_id > last_frame_id + 1)
        {
            uint32_t skipped = gaze.frame_id - last_frame_id - 1;
            missed_total += skipped;
            ESP_LOGW(TAG, "$SKIP frames=%lu total=%lu", static_cast<unsigned long>(skipped),
                     static_cast<unsigned long>(missed_total));
        }
        last_frame_id = gaze.frame_id;

        if (!gaze.human && last_human)
        {
            ESP_LOGI(TAG, "$LOST t=%lu", static_cast<unsigned long>(esp_log_timestamp()));
            log_skip = 0;
        }
        last_human = gaze.human;

        ModeFrame mf;
        build_mode_frame(gaze, mf);
        rc->frame_q->send(mf, 0);

        if (++log_skip >= 10)
        {
            log_skip = 0;
            log_frame(gaze);
        }
    }
}

} // namespace radar
