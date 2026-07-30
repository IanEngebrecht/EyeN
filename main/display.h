#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t display_init(void);

/** Fill both eyes white and reset tracked dot state. */
void display_clear(void);

/**
 * Move / resize the tracking dot on both eyes.
 * Erases the previous circle and draws a black circle at (x, y) with radius r.
 */
void display_set_dot(int x, int y, int radius);

#ifdef __cplusplus
}
#endif
