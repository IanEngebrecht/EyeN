#include "radar_stack.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "ld2450.h"

static const char *TAG = "radar_stack";

#define PI_F 3.14159265f
#define LD2450_TARGET_SLOTS 3

static const eyen_sensor_cfg_t s_cfg[] = { CFG_SENSORS };
#define SLOT_COUNT ((int)(sizeof(s_cfg) / sizeof(s_cfg[0])))

/* Indices into s_cfg sorted by ascending pitch_deg. */
static int  s_pitch_order[SLOT_COUNT];
static int  s_enabled_count;
static int  s_band_count;
static uint32_t s_frame_id;

typedef struct {
    ld2450_dev_t    *dev;
    ld2450_target_t  targets[LD2450_TARGET_SLOTS];
    TickType_t       last_ok_tick;   /* when we last got a good frame */
    bool             has_data;       /* at least one frame received   */
} slot_state_t;

static slot_state_t s_slots[SLOT_COUNT];

/* ── helpers ──────────────────────────────────────────────────────── */

static void select_mux_if_needed(int mux_channel)
{
    if (mux_channel < 0) return;
    /* TODO: drive CFG_MUX_S0_GPIO / CFG_MUX_S1_GPIO, then flush UART. */
    (void)mux_channel;
}

static bool slot_is_current(int i)
{
    if (!s_slots[i].has_data) return false;
    TickType_t age = xTaskGetTickCount() - s_slots[i].last_ok_tick;
    return age < pdMS_TO_TICKS(CFG_STALE_MS);
}

/*
 * Software-side ghost filter: reject targets that fail speed, distance,
 * or persistence checks.  Parameters are runtime-adjustable.
 */
#define PERSIST_WINDOW 3

static radar_filter_cfg_t s_filter = {
    .min_speed_cm_s = CFG_FILTER_MIN_SPEED_CM_S,
    .min_dist_mm    = CFG_FILTER_MIN_DIST_MM,
    .max_dist_mm    = CFG_FILTER_MAX_DIST_MM,
    .persist_frames = CFG_FILTER_PERSIST_FRAMES,
};

void radar_stack_get_filter(radar_filter_cfg_t *out)
{
    if (out) *out = s_filter;
}

void radar_stack_set_filter(const radar_filter_cfg_t *cfg)
{
    if (!cfg) return;
    s_filter = *cfg;
    ESP_LOGI(TAG, "filter: min_spd=%d min_d=%d max_d=%d persist=%d",
             s_filter.min_speed_cm_s, s_filter.min_dist_mm,
             s_filter.max_dist_mm, s_filter.persist_frames);
}

/* Multi-sensor backend has no potentiometer input; return neutral. */
float radar_stack_get_pot_frac(void)
{
    return 0.5f;
}

typedef struct {
    int16_t x_mm;
    int16_t y_mm;
    int     hits;
} persist_slot_t;

static persist_slot_t s_persist[SLOT_COUNT][LD2450_TARGET_SLOTS];

static void filter_targets(int slot_idx, ld2450_target_t tgt[LD2450_TARGET_SLOTS])
{
    const int min_spd = s_filter.min_speed_cm_s;
    const int min_dst = s_filter.min_dist_mm;
    const int max_dst = s_filter.max_dist_mm;
    const int req     = s_filter.persist_frames;

    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i) {
        if (!tgt[i].valid) {
            /* Decay persistence counter for this slot. */
            if (s_persist[slot_idx][i].hits > 0)
                s_persist[slot_idx][i].hits--;
            continue;
        }

        int abs_spd = tgt[i].speed_cm_s < 0
                          ? -tgt[i].speed_cm_s
                          : tgt[i].speed_cm_s;

        /* Speed gate */
        if (min_spd > 0 && abs_spd < min_spd) {
            tgt[i].valid = false;
            if (s_persist[slot_idx][i].hits > 0)
                s_persist[slot_idx][i].hits--;
            continue;
        }

        /* Distance clamp */
        if (tgt[i].distance_mm < (uint16_t)min_dst ||
            tgt[i].distance_mm > (uint16_t)max_dst) {
            tgt[i].valid = false;
            if (s_persist[slot_idx][i].hits > 0)
                s_persist[slot_idx][i].hits--;
            continue;
        }

        /* Persistence: target must have appeared in enough recent frames.
         * We match by proximity to the persisted position. */
        persist_slot_t *ps = &s_persist[slot_idx][i];
        int dx = abs(tgt[i].x_mm - ps->x_mm);
        int dy = abs(tgt[i].y_mm - ps->y_mm);
        if (dx < 600 && dy < 600) {
            if (ps->hits < PERSIST_WINDOW)
                ps->hits++;
        } else {
            ps->hits = 1;
        }
        ps->x_mm = tgt[i].x_mm;
        ps->y_mm = tgt[i].y_mm;

        if (req > 1 && ps->hits < req) {
            tgt[i].valid = false;
        }
    }
}

static bool nearest_target(const ld2450_target_t tgt[LD2450_TARGET_SLOTS],
                           ld2450_target_t *out)
{
    int best = -1;
    uint16_t best_dist = UINT16_MAX;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i) {
        if (!tgt[i].valid) continue;
        if (best < 0 || tgt[i].distance_mm < best_dist) {
            best = i;
            best_dist = tgt[i].distance_mm;
        }
    }
    if (best < 0) return false;
    *out = tgt[best];
    return true;
}

static bool match_target(const ld2450_target_t *seed,
                         const ld2450_target_t tgt[LD2450_TARGET_SLOTS],
                         ld2450_target_t *out)
{
    int best = -1, best_score = INT_MAX;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i) {
        if (!tgt[i].valid) continue;
        int adx = abs(tgt[i].x_mm - seed->x_mm);
        int add = abs((int)tgt[i].distance_mm - (int)seed->distance_mm);
        if (adx > CFG_ASSOC_MAX_DX_MM || add > CFG_ASSOC_MAX_DDIST_MM)
            continue;
        int score = adx + add;
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    if (best < 0) return false;
    *out = tgt[best];
    return true;
}

static float target_azimuth_deg(const ld2450_target_t *t)
{
    float d = fabsf((float)t->y_mm);
    if (d < 1.0f) d = 1.0f;
    return atan2f((float)t->x_mm, d) * (180.0f / PI_F);
}

/*
 * Generalized vertical band from a pitch-ordered visibility mask.
 * band = lowest_rank + highest_rank → gives 2N−1 distinct bands for N sensors.
 */
static int compute_band(uint8_t ordered_mask, int n_enabled)
{
    int lo = -1, hi = -1;
    for (int i = 0; i < n_enabled; ++i) {
        if (ordered_mask & (1u << i)) {
            if (lo < 0) lo = i;
            hi = i;
        }
    }
    return (lo < 0) ? -1 : lo + hi;
}

/* ── public API ───────────────────────────────────────────────────── */

esp_err_t radar_stack_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));

    /* Collect enabled slots into pitch-sorted order. */
    s_enabled_count = 0;
    for (int i = 0; i < SLOT_COUNT; ++i) {
        if (s_cfg[i].enabled)
            s_pitch_order[s_enabled_count++] = i;
    }

    /* Insertion sort by pitch_deg (N is tiny). */
    for (int i = 1; i < s_enabled_count; ++i) {
        int key = s_pitch_order[i];
        int j = i - 1;
        while (j >= 0 &&
               s_cfg[s_pitch_order[j]].pitch_deg > s_cfg[key].pitch_deg) {
            s_pitch_order[j + 1] = s_pitch_order[j];
            --j;
        }
        s_pitch_order[j + 1] = key;
    }

    s_band_count = (s_enabled_count > 1) ? (2 * s_enabled_count - 1) : 1;

    /* Create UART drivers. */
    for (int i = 0; i < SLOT_COUNT; ++i) {
        if (!s_cfg[i].enabled) continue;
        select_mux_if_needed(s_cfg[i].mux_channel);

        const ld2450_config_t uart = {
            .uart_num = s_cfg[i].uart_num,
            .tx_gpio  = s_cfg[i].tx_gpio,
            .rx_gpio  = s_cfg[i].rx_gpio,
            .baud     = CFG_RADAR_BAUD,
        };
        esp_err_t err = ld2450_create(&uart, &s_slots[i].dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "create %s failed: %s",
                     s_cfg[i].name, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "slot %-6s  pitch=%+3d°  inv=%d  uart=%d  rx=%d  tx=%d",
                 s_cfg[i].name, s_cfg[i].pitch_deg,
                 s_cfg[i].inverted, (int)s_cfg[i].uart_num,
                 s_cfg[i].rx_gpio, s_cfg[i].tx_gpio);

        /* Unstick sensor if it was left in config mode from a previous session. */
        if (s_cfg[i].tx_gpio >= 0) {
            ld2450_unstick(s_slots[i].dev);
            char ver[32];
            if (ld2450_read_firmware_version(s_slots[i].dev, ver, sizeof(ver)) == ESP_OK)
                ESP_LOGI(TAG, "  %s firmware: %s", s_cfg[i].name, ver);
            else
                ESP_LOGW(TAG, "  %s firmware: could not read (TX wired?)", s_cfg[i].name);
        }
    }

    ESP_LOGI(TAG, "%d sensor(s) enabled, %d vertical band(s)",
             s_enabled_count, s_band_count);
    return ESP_OK;
}

ld2450_dev_t *radar_stack_get_dev(int slot)
{
    if (slot < 0 || slot >= s_enabled_count) return NULL;
    return s_slots[s_pitch_order[slot]].dev;
}

int radar_stack_slot_count(void)
{
    return s_enabled_count;
}

static int count_valid(const ld2450_target_t tgt[LD2450_TARGET_SLOTS])
{
    int n = 0;
    for (int i = 0; i < LD2450_TARGET_SLOTS; ++i)
        if (tgt[i].valid) ++n;
    return n;
}

esp_err_t radar_stack_update(radar_gaze_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));
    out->vertical_band = -1;
    out->band_count = s_band_count;

    /*
     * 1. Poll every enabled slot.  On success, overwrite stored targets.
     *    On timeout, keep the previous frame — it stays valid until
     *    CFG_STALE_MS elapses.
     */
    for (int i = 0; i < SLOT_COUNT; ++i) {
        if (!s_cfg[i].enabled || !s_slots[i].dev) continue;

        ld2450_target_t tmp[LD2450_TARGET_SLOTS];
        memset(tmp, 0, sizeof(tmp));

        select_mux_if_needed(s_cfg[i].mux_channel);
        esp_err_t err = ld2450_read_frame(s_slots[i].dev, tmp,
                                          CFG_FRAME_TIMEOUT_MS);
        if (err == ESP_OK) {
            if (s_cfg[i].inverted) {
                for (int t = 0; t < LD2450_TARGET_SLOTS; ++t)
                    tmp[t].x_mm = -tmp[t].x_mm;
            }
            filter_targets(i, tmp);
            memcpy(s_slots[i].targets, tmp, sizeof(tmp));
            s_slots[i].last_ok_tick = xTaskGetTickCount();
            s_slots[i].has_data = true;
            s_frame_id++;
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "%s frame: %s", s_cfg[i].name, esp_err_to_name(err));
        }
    }

    out->frame_id = s_frame_id;

    /* 1b. Fill per-slot info (all targets) for the caller. */
    int si = 0;
    int total = 0;
    for (int rank = 0; rank < s_enabled_count; ++rank) {
        int i = s_pitch_order[rank];
        radar_slot_info_t *info = &out->slots[si++];
        info->name = s_cfg[i].name;
        if (slot_is_current(i)) {
            memcpy(info->targets, s_slots[i].targets, sizeof(info->targets));
            info->target_count = count_valid(s_slots[i].targets);
        }
        total += info->target_count;
    }
    out->slot_count = si;
    out->total_targets = total;

    /* 2. Seed: nearest target across all slots with current data. */
    int seed_slot = -1;
    ld2450_target_t seed = {0};
    uint16_t best_dist = UINT16_MAX;
    for (int i = 0; i < SLOT_COUNT; ++i) {
        if (!s_cfg[i].enabled || !slot_is_current(i)) continue;
        ld2450_target_t cand;
        if (!nearest_target(s_slots[i].targets, &cand)) continue;
        if (seed_slot < 0 || cand.distance_mm < best_dist) {
            seed_slot = i;
            seed      = cand;
            best_dist = cand.distance_mm;
        }
    }
    if (seed_slot < 0) {
        out->human = false;
        return ESP_OK;
    }

    /* 3. Associate across current slots; build pitch-ordered visibility mask. */
    uint8_t ordered_mask = 0;
    float   az_sum = 0.0f;
    int     az_n   = 0;

    for (int rank = 0; rank < s_enabled_count; ++rank) {
        int i = s_pitch_order[rank];
        if (!slot_is_current(i)) continue;

        ld2450_target_t m;
        bool ok;
        if (i == seed_slot) {
            m  = seed;
            ok = true;
        } else {
            ok = match_target(&seed, s_slots[i].targets, &m);
        }
        if (!ok) continue;

        ordered_mask |= (uint8_t)(1u << rank);
        az_sum += target_azimuth_deg(&m);
        az_n++;
    }

    out->human         = (az_n > 0);
    out->see_mask      = ordered_mask;
    out->primary       = seed;
    out->vertical_band = compute_band(ordered_mask, s_enabled_count);
    out->azimuth_deg   = (az_n > 0) ? (az_sum / (float)az_n) : 0.0f;

    if (out->vertical_band < 0 || s_band_count <= 1) {
        out->elevation_norm = 0.5f;
    } else {
        out->elevation_norm =
            (float)out->vertical_band / (float)(s_band_count - 1);
    }
    return ESP_OK;
}
