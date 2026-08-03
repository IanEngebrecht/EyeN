#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t display_init(void);

esp_lcd_panel_handle_t display_get_panel_left(void);
esp_lcd_panel_handle_t display_get_panel_right(void);

void display_fill(esp_lcd_panel_handle_t panel, uint16_t color);

#ifdef __cplusplus
}
#endif
