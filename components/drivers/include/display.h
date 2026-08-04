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

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t display_init(void);

    esp_lcd_panel_handle_t display_get_panel_left(void);
    esp_lcd_panel_handle_t display_get_panel_right(void);

    void display_fill(esp_lcd_panel_handle_t panel, uint16_t color);

#ifdef __cplusplus
}
#endif
