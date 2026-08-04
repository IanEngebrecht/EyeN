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

#include "ld2450.h"

namespace radar
{

struct PersistSlot
{
    int16_t x_mm{};
    int16_t y_mm{};
    int hits{};
};

struct FilterConfig
{
    int min_speed_cm_s{};
    int min_dist_mm{};
    int max_dist_mm{};
    int persist_frames{};
};

void filter_targets(std::span<Target> tgt, std::span<PersistSlot> persist, const FilterConfig &cfg);
float target_azimuth_deg(const Target &t);
int count_valid(std::span<const Target> tgt);
int nearest_target_idx(std::span<const Target> tgt);

} // namespace radar
