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

#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "ld2450.h"

static const char *TAG = "radar_single";

#define PI_F                3.14159265f
#define LD2450_TARGET_SLOTS 3
#define PERSIST_WINDOW      3

static const eyen_sensor_cfg_t s_cfg[] = {CFG_SENSORS};
#define SLOT_COUNT ((int)(sizeof(s_cfg) / sizeof(s_cfg[0])))

static int s_slot = -1; /* index of first enabled sensor */
static ld2450_dev_t *s_dev;
static ld2450_target_t s_targets[LD2450_TARGET_SLOTS];
static TickType_t s_last_ok_tick;
static bool s_has_data;
static uint32_t s_frame_id;

static adc_oneshot_unit_handle_t s_adc;
static float s_pot_filt; /* smoothed ADC 0‥1 */
static bool s_pot_ok;

/* Multi-target attention: which radar slot we're looking at + dwell timer */
static int s_focus_idx = -1;
static TickType_t s_hold_deadline;
static TickType_t s_motion_lock_until; /* block mover↔mover thrashing */

static radar_filter_cfg_t s_filter = {
    .min_speed_cm_s = CFG_FILTER_MIN_SPEED_CM_S,
    .min_dist_mm = CFG_FILTER_MIN_DIST_MM,
    .max_dist_mm = CFG_FILTER_MAX_DIST_MM,
    .persist_frames = CFG_FILTER_PERSIST_FRAMES,
};

typedef struct
{
    int16_t x_mm;
    int16_t y_mm;
    int hits;
} persist_slot_t;

static persist_slot_t s_persist[LD2450_TARGET_SLOTS];

void radar_stack_get_filter(radar_filter_cfg_t *out)
{
    if (out)
        *out = s_filter;
}

void radar_stack_set_filter(const radar_filter_cfg_t *cfg)
{
    if (!cfg)
        return;
    s_filter = *cfg;
    ESP_LOGI(TAG, "filter: min_spd=%d min_d=%d max_d=%d persist=%d", s_filter.min_speed_cm_s,
             s_filter.min_dist_mm, s_filter.max_dist_mm, s_filter.persist_frames);
}

static void filter_targets(ld2450_target_t tgt[LD2450_TARGET_SLOTS])
{
    const int min_spd = s_filter.min_speed_cm_s;
    const int min_dst = s_filter.min_dist_mm;
    const int max_dst = s_filter.max_dist_mm;
    const int req = s_filter.persist_frames;

    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i)
    {
        if (!tgt[i].valid)
        {
            if (s_persist[i].hits > 0)
                s_persist[i].hits--;
            continue;
        }

        int abs_spd = tgt[i].speed_cm_s < 0 ? -tgt[i].speed_cm_s : tgt[i].speed_cm_s;

        if (min_spd > 0 && abs_spd < min_spd)
        {
            tgt[i].valid = false;
            if (s_persist[i].hits > 0)
                s_persist[i].hits--;
            continue;
        }

        if (tgt[i].distance_mm < (uint16_t)min_dst || tgt[i].distance_mm > (uint16_t)max_dst)
        {
            tgt[i].valid = false;
            if (s_persist[i].hits > 0)
                s_persist[i].hits--;
            continue;
        }

        persist_slot_t *ps = &s_persist[i];
        int dx = abs(tgt[i].x_mm - ps->x_mm);
        int dy = abs(tgt[i].y_mm - ps->y_mm);
        if (dx < 600 && dy < 600)
        {
            if (ps->hits < PERSIST_WINDOW)
                ps->hits++;
        }
        else
        {
            ps->hits = 1;
        }
        ps->x_mm = tgt[i].x_mm;
        ps->y_mm = tgt[i].y_mm;

        if (req > 1 && ps->hits < req)
            tgt[i].valid = false;
    }
}

static int nearest_target_idx(const ld2450_target_t tgt[LD2450_TARGET_SLOTS])
{
    int best = -1;
    uint16_t best_dist = UINT16_MAX;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i)
    {
        if (!tgt[i].valid)
            continue;
        if (best < 0 || tgt[i].distance_mm < best_dist)
        {
            best = i;
            best_dist = tgt[i].distance_mm;
        }
    }
    return best;
}

static int abs_speed_cm_s(const ld2450_target_t *t)
{
    return t->speed_cm_s < 0 ? -t->speed_cm_s : t->speed_cm_s;
}

static TickType_t random_hold_ticks(void)
{
    const uint32_t span = (uint32_t)(CFG_GAZE_HOLD_MAX_MS - CFG_GAZE_HOLD_MIN_MS);
    uint32_t ms = (uint32_t)CFG_GAZE_HOLD_MIN_MS;
    if (span > 0)
        ms += esp_random() % (span + 1);
    return pdMS_TO_TICKS(ms);
}

static void arm_hold(void)
{
    s_hold_deadline = xTaskGetTickCount() + random_hold_ticks();
}

/*
 * Choose which target the eyes should look at.
 * 0 targets → false
 * 1 target  → that one (same as before)
 * 2–3       → dwell 2–5 s then rotate while all stationary;
 *             motion steals focus, but stick to the current mover and
 *             lock out other movers for CFG_GAZE_MOTION_LOCK_MS to avoid thrash.
 */
static bool select_attention_target(const ld2450_target_t tgt[LD2450_TARGET_SLOTS],
                                    ld2450_target_t *out)
{
    int valid[LD2450_TARGET_SLOTS];
    int n = 0;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i)
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
        *out = tgt[s_focus_idx];
        return true;
    }

    const TickType_t now = xTaskGetTickCount();
    const bool focus_valid =
        s_focus_idx >= 0 && s_focus_idx < LD2450_TARGET_SLOTS && tgt[s_focus_idx].valid;
    const bool focus_moving =
        focus_valid && abs_speed_cm_s(&tgt[s_focus_idx]) >= CFG_GAZE_MOTION_CM_S;

    /* Sticky: while our person is still moving, stay on them. */
    if (focus_moving)
    {
        arm_hold();
        *out = tgt[s_focus_idx];
        return true;
    }

    /* Find fastest mover (excluding a locked previous focus handled above). */
    int mover = -1;
    int mover_spd = CFG_GAZE_MOTION_CM_S - 1;
    for (int k = 0; k < n; ++k)
    {
        int i = valid[k];
        int spd = abs_speed_cm_s(&tgt[i]);
        if (spd >= CFG_GAZE_MOTION_CM_S && spd > mover_spd)
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
                s_motion_lock_until = now + pdMS_TO_TICKS(CFG_GAZE_MOTION_LOCK_MS);
            }
            s_focus_idx = mover;
            arm_hold();
            *out = tgt[s_focus_idx];
            return true;
        }
        /* Locked onto a stationary focus while others move: keep hold. */
        arm_hold();
        *out = tgt[s_focus_idx];
        return true;
    }

    /* All stationary: keep focus if still valid, else pick nearest. */
    if (!focus_valid)
    {
        s_focus_idx = nearest_target_idx(tgt);
        if (s_focus_idx < 0)
            return false;
        arm_hold();
        *out = tgt[s_focus_idx];
        return true;
    }

    /* Dwell expired → advance to next valid slot (wrap). */
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

    *out = tgt[s_focus_idx];
    return true;
}

static float target_azimuth_deg(const ld2450_target_t *t)
{
    float d = fabsf((float)t->y_mm);
    if (d < 1.0f)
        d = 1.0f;
    return atan2f((float)t->x_mm, d) * (180.0f / PI_F);
}

static int count_valid(const ld2450_target_t tgt[LD2450_TARGET_SLOTS])
{
    int n = 0;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i)
        if (tgt[i].valid)
            ++n;
    return n;
}

static bool slot_is_current(void)
{
    if (!s_has_data)
        return false;
    TickType_t age = xTaskGetTickCount() - s_last_ok_tick;
    return age < pdMS_TO_TICKS(CFG_STALE_MS);
}

static esp_err_t pot_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = CFG_POT_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC unit: %s (pot disabled)", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = CFG_POT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, CFG_POT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC channel: %s (pot disabled)", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return err;
    }

    s_pot_ok = true;
    s_pot_filt = 0.5f;
    ESP_LOGI(TAG, "pot on GPIO%d (ADC), mount %d‥%d mm", CFG_POT_GPIO, CFG_POT_MOUNT_MIN_MM,
             CFG_POT_MOUNT_MAX_MM);
    return ESP_OK;
}

/** Sample the pot ADC and update the smoothed 0‥1 fraction. Called once per
    radar_stack_update(), independent of whether a human is being tracked, so
    mode-switch logic in main.c always sees a fresh reading. */
static void pot_update(void)
{
    if (!s_pot_ok || !s_adc)
        return;

    int raw = 0;
    if (adc_oneshot_read(s_adc, CFG_POT_ADC_CHANNEL, &raw) != ESP_OK)
        return;

    /* 12-bit ADC: 0‥4095 typical with default bitwidth. */
    float n = (float)raw / 4095.0f;
    if (n < 0.0f)
        n = 0.0f;
    if (n > 1.0f)
        n = 1.0f;
    s_pot_filt += (n - s_pot_filt) * CFG_POT_SMOOTH;
}

float radar_stack_get_pot_frac(void)
{
    return s_pot_ok ? s_pot_filt : 0.5f;
}

/** Pot fraction → mount height in mm (person_aim − height drives elevation). */
static float pot_mount_height_mm(void)
{
    return CFG_POT_MOUNT_MIN_MM +
           radar_stack_get_pot_frac() * (float)(CFG_POT_MOUNT_MAX_MM - CFG_POT_MOUNT_MIN_MM);
}

/**
 * θ = atan2(person_aim − mount_height, range_y)
 * Map θ through ±CFG_GAZE_MAX_DEG into elevation_norm [0,1].
 */
static float elevation_from_distance(const ld2450_target_t *t)
{
    float y = fabsf((float)t->y_mm);
    if (y < 1.0f)
        y = 1.0f;

    float dh = (float)CFG_PERSON_AIM_MM - pot_mount_height_mm();
    float elev_deg = atan2f(dh, y) * (180.0f / PI_F);

    float deg = elev_deg;
    if (deg < -CFG_GAZE_MAX_DEG)
        deg = -CFG_GAZE_MAX_DEG;
    else if (deg > CFG_GAZE_MAX_DEG)
        deg = CFG_GAZE_MAX_DEG;

    /* +elev (look up) → pupil toward top (1); -elev → bottom (0). */
    return (deg + CFG_GAZE_MAX_DEG) / (2.0f * CFG_GAZE_MAX_DEG);
}

esp_err_t radar_stack_init(void)
{
    s_dev = NULL;
    s_has_data = false;
    s_slot = -1;
    s_focus_idx = -1;
    s_hold_deadline = 0;
    s_motion_lock_until = 0;
    memset(s_targets, 0, sizeof(s_targets));
    memset(s_persist, 0, sizeof(s_persist));

    for (int i = 0; i < SLOT_COUNT; ++i)
    {
        if (s_cfg[i].enabled)
        {
            s_slot = i;
            break;
        }
    }
    if (s_slot < 0)
    {
        ESP_LOGE(TAG, "no enabled sensor in CFG_SENSORS");
        return ESP_ERR_INVALID_STATE;
    }

    const eyen_sensor_cfg_t *cfg = &s_cfg[s_slot];
    const ld2450_config_t uart = {
        .uart_num = cfg->uart_num,
        .tx_gpio = cfg->tx_gpio,
        .rx_gpio = cfg->rx_gpio,
        .baud = CFG_RADAR_BAUD,
    };
    esp_err_t err = ld2450_create(&uart, &s_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "create %s failed: %s", cfg->name, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "sensor %-6s  pitch=%+3d°  inv=%d  uart=%d  rx=%d  tx=%d", cfg->name,
             cfg->pitch_deg, cfg->inverted, (int)cfg->uart_num, cfg->rx_gpio, cfg->tx_gpio);

    if (cfg->tx_gpio >= 0)
    {
        ld2450_unstick(s_dev);
        char ver[32];
        if (ld2450_read_firmware_version(s_dev, ver, sizeof(ver)) == ESP_OK)
            ESP_LOGI(TAG, "  %s firmware: %s", cfg->name, ver);
        else
            ESP_LOGW(TAG, "  %s firmware: could not read (TX wired?)", cfg->name);
    }

    pot_init(); /* soft-fail: elevation uses mid-range mount if ADC fails */
    return ESP_OK;
}

ld2450_dev_t *radar_stack_get_dev(int slot)
{
    if (slot != 0 || !s_dev)
        return NULL;
    return s_dev;
}

int radar_stack_slot_count(void)
{
    return (s_dev != NULL) ? 1 : 0;
}

esp_err_t radar_stack_update(radar_gaze_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));
    out->vertical_band = -1;
    out->band_count = 1;
    out->elevation_norm = 0.5f;

    pot_update();

    if (!s_dev || s_slot < 0)
        return ESP_ERR_INVALID_STATE;

    const eyen_sensor_cfg_t *cfg = &s_cfg[s_slot];

    ld2450_target_t tmp[LD2450_TARGET_SLOTS];
    memset(tmp, 0, sizeof(tmp));
    esp_err_t err = ld2450_read_frame(s_dev, tmp, CFG_FRAME_TIMEOUT_MS);
    if (err == ESP_OK)
    {
        if (cfg->inverted)
        {
            for (int t = 0; t < LD2450_TARGET_SLOTS; ++t)
                tmp[t].x_mm = -tmp[t].x_mm;
        }
        filter_targets(tmp);
        memcpy(s_targets, tmp, sizeof(tmp));
        s_last_ok_tick = xTaskGetTickCount();
        s_has_data = true;
        s_frame_id++;
    }
    else if (err != ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "%s frame: %s", cfg->name, esp_err_to_name(err));
    }

    out->frame_id = s_frame_id;

    radar_slot_info_t *info = &out->slots[0];
    info->name = cfg->name;
    if (slot_is_current())
    {
        memcpy(info->targets, s_targets, sizeof(info->targets));
        info->target_count = count_valid(s_targets);
    }
    out->slot_count = 1;
    out->total_targets = info->target_count;

    if (!slot_is_current())
    {
        out->human = false;
        return ESP_OK;
    }

    ld2450_target_t focus;
    if (!select_attention_target(s_targets, &focus))
    {
        out->human = false;
        return ESP_OK;
    }

    out->human = true;
    out->primary = focus;
    out->azimuth_deg = target_azimuth_deg(&focus);
    out->elevation_norm = elevation_from_distance(&focus);
    out->see_mask = 0x01;
    return ESP_OK;
}
