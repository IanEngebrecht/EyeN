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

#include <array>
#include <cstdint>

#include "esp_lcd_panel_ops.h"
#include "ld2450.h"

struct ModeFrame
{
    std::array<Target, Ld2450::target_count> targets{};
    int target_count{};
    int primary_idx{-1};
    Target primary{};
    float azimuth_deg{};
    float elevation_norm{};
    bool human{};
    uint32_t frame_id{};
};

class DisplayMode
{
  public:
    virtual ~DisplayMode() = default;
    virtual const char *name() const = 0;
    virtual void enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right) = 0;
    virtual void render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                        const ModeFrame &frame) = 0;
    virtual void leave() = 0;
};

DisplayMode &eye_mode();
DisplayMode &radar_mode();
