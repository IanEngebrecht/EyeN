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

#ifdef __cplusplus
}
#endif
