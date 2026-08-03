#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_lcd_panel_ops.h"
#include "ld2450.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MODE_MAX_TARGETS 3

typedef struct {
    ld2450_target_t targets[MODE_MAX_TARGETS];
    int             target_count;
    int             primary_idx;       /* -1 if none tracked */
    ld2450_target_t primary;
    float           azimuth_deg;
    float           elevation_norm;
    bool            human;
    uint32_t        frame_id;
} mode_frame_t;

typedef struct {
    const char *name;
    void (*enter)(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right);
    void (*render)(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                   const mode_frame_t *frame);
    void (*leave)(void);
} display_mode_t;

extern const display_mode_t mode_eye;
extern const display_mode_t mode_radar;

#ifdef __cplusplus
}
#endif
