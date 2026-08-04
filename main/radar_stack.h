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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ld2450.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_MAX_SLOTS        4
#define RADAR_TARGETS_PER_SLOT 3

typedef struct {
    const char     *name;
    int             target_count;
    ld2450_target_t targets[RADAR_TARGETS_PER_SLOT];
} radar_slot_info_t;

typedef struct {
    bool    human;
    float   azimuth_deg;
    float   elevation_norm; /* 0 = pupil bottom, 1 = top */
    int     vertical_band;  /* multi backend; -1 when unused */
    int     band_count;
    uint8_t see_mask;

    ld2450_target_t primary;

    /* Bumps only when a new UART radar frame was parsed (not on stale reuse). */
    uint32_t         frame_id;

    int              total_targets;
    int              slot_count;
    radar_slot_info_t slots[RADAR_MAX_SLOTS];
} radar_gaze_t;

esp_err_t radar_stack_init(void);
esp_err_t radar_stack_update(radar_gaze_t *out);

/** Get the LD2450 device handle for a slot index (for sending config commands). */
ld2450_dev_t *radar_stack_get_dev(int slot);
int radar_stack_slot_count(void);

/** Runtime-adjustable software filter parameters. */
typedef struct {
    int min_speed_cm_s;
    int min_dist_mm;
    int max_dist_mm;
    int persist_frames;
} radar_filter_cfg_t;

void radar_stack_get_filter(radar_filter_cfg_t *out);
void radar_stack_set_filter(const radar_filter_cfg_t *cfg);

/** Smoothed potentiometer position, 0.0‥1.0 (0.5 if unavailable/unused). */
float radar_stack_get_pot_frac(void);

#ifdef __cplusplus
}
#endif
