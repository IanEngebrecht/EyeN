#pragma once

#include <array>
#include <cstdint>

#include "target.h"

inline constexpr int mode_frame_max_targets = 3;

struct ModeFrame
{
    std::array<Target, mode_frame_max_targets> targets{};
    int target_count{};
    int primary_idx{-1};
    Target primary{};
    float azimuth_deg{};
    float elevation_norm{};
    bool human{};
    uint32_t frame_id{};
};
