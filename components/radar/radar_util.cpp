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

#include "radar_util.h"

#include <climits>
#include <cmath>
#include <cstdlib>
#include <numbers>

namespace radar
{

namespace
{

constexpr int persist_window = 3;
constexpr int proximity_mm = 600;

} // anonymous namespace

void filter_targets(std::span<Target> tgt, std::span<PersistSlot> persist, const FilterConfig &cfg)
{
    const int min_spd = cfg.min_speed_cm_s;
    const int min_dst = cfg.min_dist_mm;
    const int max_dst = cfg.max_dist_mm;
    const int req = cfg.persist_frames;

    for (size_t i = 0; i < tgt.size(); ++i)
    {
        if (!tgt[i].valid)
        {
            if (i < persist.size() && persist[i].hits > 0)
                persist[i].hits--;
            continue;
        }

        int abs_spd = std::abs(tgt[i].speed_cm_s);

        if (min_spd > 0 && abs_spd < min_spd)
        {
            tgt[i].valid = false;
            if (i < persist.size() && persist[i].hits > 0)
                persist[i].hits--;
            continue;
        }

        if (tgt[i].distance_mm < static_cast<uint16_t>(min_dst) ||
            tgt[i].distance_mm > static_cast<uint16_t>(max_dst))
        {
            tgt[i].valid = false;
            if (i < persist.size() && persist[i].hits > 0)
                persist[i].hits--;
            continue;
        }

        if (i < persist.size())
        {
            PersistSlot &ps = persist[i];
            int dx = std::abs(tgt[i].x_mm - ps.x_mm);
            int dy = std::abs(tgt[i].y_mm - ps.y_mm);
            if (dx < proximity_mm && dy < proximity_mm)
            {
                if (ps.hits < persist_window)
                    ps.hits++;
            }
            else
            {
                ps.hits = 1;
            }
            ps.x_mm = tgt[i].x_mm;
            ps.y_mm = tgt[i].y_mm;

            if (req > 1 && ps.hits < req)
                tgt[i].valid = false;
        }
    }
}

float target_azimuth_deg(const Target &t)
{
    float d = std::fabs(static_cast<float>(t.y_mm));
    if (d < 1.0f)
        d = 1.0f;
    return std::atan2(static_cast<float>(t.x_mm), d) * (180.0f / std::numbers::pi_v<float>);
}

int count_valid(std::span<const Target> tgt)
{
    int n = 0;
    for (const auto &t : tgt)
        if (t.valid)
            ++n;
    return n;
}

int nearest_target_idx(std::span<const Target> tgt)
{
    int best = -1;
    uint16_t best_dist = UINT16_MAX;
    for (size_t i = 0; i < tgt.size(); ++i)
    {
        if (!tgt[i].valid)
            continue;
        if (best < 0 || tgt[i].distance_mm < best_dist)
        {
            best = static_cast<int>(i);
            best_dist = tgt[i].distance_mm;
        }
    }
    return best;
}

} // namespace radar
