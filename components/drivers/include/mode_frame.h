#pragma once

#include <array>
#include <cstdint>

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
