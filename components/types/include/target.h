#pragma once

#include <cstdint>

struct Target
{
    int16_t x_mm{};
    int16_t y_mm{};
    int16_t speed_cm_s{};
    uint16_t distance_mm{};
    bool valid{};
};
