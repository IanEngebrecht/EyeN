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

#include <cstdint>
#include <span>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

#include "config.h"

class Display
{
  public:
    esp_err_t init();

    esp_lcd_panel_handle_t left() const { return left_; }
    esp_lcd_panel_handle_t right() const { return right_; }

    void fill(esp_lcd_panel_handle_t panel, uint16_t color);

    std::span<uint16_t, cfg::lcd::h_res> scanline_buf() { return dma_line_; }

  private:
    esp_lcd_panel_handle_t left_{};
    esp_lcd_panel_handle_t right_{};

    alignas(4) uint16_t dma_line_[cfg::lcd::h_res]{};

    static esp_err_t apply_rotation(esp_lcd_panel_handle_t panel, int deg);
    esp_err_t create_panel(spi_host_device_t host, int cs_gpio, int rotation_deg,
                           esp_lcd_panel_handle_t *out);
    static esp_err_t pulse_shared_reset();
};
