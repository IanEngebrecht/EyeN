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

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "mode_frame.h"
#include "radar_util.h"
#include "rtos/queue.h"

namespace radar
{

inline constexpr int max_slots = 4;

struct SlotInfo
{
    const char *name{};
    int target_count{};
    Target targets[mode_frame_max_targets]{};
};

struct Gaze
{
    bool human{};
    float azimuth_deg{};
    float elevation_norm{0.5f};
    int vertical_band{-1};
    int band_count{1};
    uint8_t see_mask{};

    Target primary{};
    uint32_t frame_id{};

    int total_targets{};
    int slot_count{};
    SlotInfo slots[max_slots]{};
};

struct DevCommand
{
    enum class Kind : uint8_t
    {
        filter_min_speed,
        filter_min_dist,
        filter_max_dist,
        filter_persist,
        hw_sensitivity,
        hw_energy,
        hw_speed_filter,
        hw_hold_time,
        hw_restart,
    };
    Kind kind;
    int value;
};

inline constexpr int frame_queue_depth = 2;
inline constexpr int cmd_queue_depth = 4;

struct RunCtx
{
    rtos::Queue<ModeFrame, frame_queue_depth> *frame_q;
    rtos::Queue<DevCommand, cmd_queue_depth> *cmd_q;
    std::atomic<float> *pot_frac;
};

esp_err_t init();
[[noreturn]] void run(void *ctx);

} // namespace radar
