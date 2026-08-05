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

#include "mode.h"

#include <algorithm>
#include <cmath>
#include <span>

#include "esp_log.h"

#include "colors.h"
#include "config.h"

static const char *TAG = "mode_eye";

namespace
{

void clamp_to_circle(float &x, float &y)
{
    const float cx = (cfg::lcd::h_res - 1) * 0.5f;
    const float cy = (cfg::lcd::v_res - 1) * 0.5f;
    const float max_r = (cfg::lcd::h_res * 0.5f) - static_cast<float>(cfg::dot::radius_max) - 2.0f;
    const float dx = x - cx;
    const float dy = y - cy;
    const float dist = std::sqrt(dx * dx + dy * dy);
    if (dist > max_r && dist > 0.0f)
    {
        const float s = max_r / dist;
        x = cx + dx * s;
        y = cy + dy * s;
    }
}

float radius_from_distance(uint16_t dist_mm)
{
    float d = static_cast<float>(dist_mm);
    if (d <= static_cast<float>(cfg::dot::near_mm))
        return static_cast<float>(cfg::dot::radius_max);
    if (d >= static_cast<float>(cfg::dot::far_mm))
        return static_cast<float>(cfg::dot::radius_min);
    float t = (d - static_cast<float>(cfg::dot::near_mm)) /
              static_cast<float>(cfg::dot::far_mm - cfg::dot::near_mm);
    return static_cast<float>(cfg::dot::radius_max) +
           t * static_cast<float>(cfg::dot::radius_min - cfg::dot::radius_max);
}

void gaze_xy(float az_deg, float elev_norm, float &tx, float &ty)
{
    float deg = az_deg;
    if (deg < -cfg::gaze::max_deg)
        deg = -cfg::gaze::max_deg;
    else if (deg > cfg::gaze::max_deg)
        deg = cfg::gaze::max_deg;

    tx = ((deg + cfg::gaze::max_deg) / (2.0f * cfg::gaze::max_deg)) *
         static_cast<float>(cfg::lcd::h_res - 1);

    float elev = elev_norm;
    if (elev < 0.0f)
        elev = 0.0f;
    else if (elev > 1.0f)
        elev = 1.0f;
    ty = (1.0f - elev) * static_cast<float>(cfg::lcd::v_res - 1);
    clamp_to_circle(tx, ty);
}

} // anonymous namespace

class EyeMode : public DisplayMode
{
  public:
    const char *name() const override { return "eye"; }
    void enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
               std::span<uint16_t> scanline) override;
    void render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                const ModeFrame &frame) override;
    void leave() override;

  private:
    esp_lcd_panel_handle_t left_{};
    esp_lcd_panel_handle_t right_{};
    std::span<uint16_t> scanline_{};
    int dot_x_{-1};
    int dot_y_{-1};
    int dot_r_{};
    bool dot_valid_{};
    float cur_x_{};
    float cur_y_{};
    float cur_r_{};

    void move_dot_on_panel(esp_lcd_panel_handle_t panel, int ox, int oy, int orad, int nx, int ny,
                           int nrad);
    void draw_filled_circle(esp_lcd_panel_handle_t panel, int cx, int cy, int radius,
                            uint16_t color);
    void set_dot(int x, int y, int radius);
};

void EyeMode::move_dot_on_panel(esp_lcd_panel_handle_t panel, int ox, int oy, int orad, int nx,
                                int ny, int nrad)
{
    const int nr2 = nrad * nrad;
    const int margin = 2;

    int bx0 = (ox - orad < nx - nrad ? ox - orad : nx - nrad) - margin;
    int bx1 = (ox + orad > nx + nrad ? ox + orad : nx + nrad) + margin;
    int by0 = (oy - orad < ny - nrad ? oy - orad : ny - nrad) - margin;
    int by1 = (oy + orad > ny + nrad ? oy + orad : ny + nrad) + margin;

    if (bx0 < 0)
        bx0 = 0;
    if (bx1 >= cfg::lcd::h_res)
        bx1 = cfg::lcd::h_res - 1;
    if (by0 < 0)
        by0 = 0;
    if (by1 >= cfg::lcd::v_res)
        by1 = cfg::lcd::v_res - 1;

    const int w = bx1 - bx0 + 1;
    for (int y = by0; y <= by1; ++y)
    {
        for (int i = 0; i < w; ++i)
        {
            int x = bx0 + i;
            int dx = x - nx, dy = y - ny;
            scanline_[i] = (dx * dx + dy * dy <= nr2) ? colors::black : colors::white;
        }
        esp_lcd_panel_draw_bitmap(panel, bx0, y, bx1 + 1, y + 1, scanline_.data());
    }
}

void EyeMode::draw_filled_circle(esp_lcd_panel_handle_t panel, int cx, int cy, int radius,
                                 uint16_t color)
{
    const int r2 = radius * radius;
    const int x0 = cx - radius;
    const int x1 = cx + radius;
    const int y0 = cy - radius;
    const int y1 = cy + radius;

    for (int y = y0; y <= y1; ++y)
    {
        if (y < 0 || y >= cfg::lcd::v_res)
            continue;
        int draw_x0 = -1, draw_x1 = -1;
        for (int x = x0; x <= x1; ++x)
        {
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r2 && x >= 0 && x < cfg::lcd::h_res)
            {
                if (draw_x0 < 0)
                    draw_x0 = x;
                draw_x1 = x;
            }
        }
        if (draw_x0 < 0)
            continue;
        const int n = draw_x1 - draw_x0 + 1;
        for (int i = 0; i < n; ++i)
            scanline_[i] = color;
        esp_lcd_panel_draw_bitmap(panel, draw_x0, y, draw_x1 + 1, y + 1, scanline_.data());
    }
}

void EyeMode::set_dot(int x, int y, int radius)
{
    if (radius < 1)
        radius = 1;
    if (radius > cfg::dot::radius_max)
        radius = cfg::dot::radius_max;
    if (x < radius)
        x = radius;
    else if (x >= cfg::lcd::h_res - radius)
        x = cfg::lcd::h_res - radius - 1;
    if (y < radius)
        y = radius;
    else if (y >= cfg::lcd::v_res - radius)
        y = cfg::lcd::v_res - radius - 1;

    if (dot_valid_ && dot_x_ == x && dot_y_ == y && dot_r_ == radius)
        return;

    if (dot_valid_)
    {
        move_dot_on_panel(left_, dot_x_, dot_y_, dot_r_, x, y, radius);
        move_dot_on_panel(right_, dot_x_, dot_y_, dot_r_, x, y, radius);
    }
    else
    {
        draw_filled_circle(left_, x, y, radius, colors::black);
        draw_filled_circle(right_, x, y, radius, colors::black);
    }
    dot_x_ = x;
    dot_y_ = y;
    dot_r_ = radius;
    dot_valid_ = true;
}

void EyeMode::enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                    std::span<uint16_t> scanline)
{
    left_ = left;
    right_ = right;
    scanline_ = scanline;
    dot_valid_ = false;
    dot_x_ = -1;
    dot_y_ = -1;
    dot_r_ = 0;
    cur_x_ = static_cast<float>(cfg::lcd::h_res / 2);
    cur_y_ = static_cast<float>(cfg::lcd::v_res / 2);
    cur_r_ = static_cast<float>(cfg::dot::radius_min);

    fill_panel(left_, colors::white, scanline_);
    fill_panel(right_, colors::white, scanline_);

    set_dot(static_cast<int>(cur_x_), static_cast<int>(cur_y_), static_cast<int>(cur_r_));
    ESP_LOGI(TAG, "entered eye mode");
}

void EyeMode::render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                     const ModeFrame &frame)
{
    (void)left;
    (void)right;

    float tgt_x, tgt_y, tgt_r;
    if (frame.human)
    {
        gaze_xy(frame.azimuth_deg, frame.elevation_norm, tgt_x, tgt_y);
        tgt_r = radius_from_distance(frame.primary.distance_mm);
    }
    else
    {
        tgt_x = static_cast<float>(cfg::lcd::h_res / 2);
        tgt_y = static_cast<float>(cfg::lcd::v_res / 2);
        tgt_r = static_cast<float>(cfg::dot::radius_min);
    }

    cur_x_ += (tgt_x - cur_x_) * cfg::smooth::h;
    cur_y_ += (tgt_y - cur_y_) * cfg::smooth::v;
    cur_r_ += (tgt_r - cur_r_) * cfg::smooth::r;

    set_dot(static_cast<int>(std::lround(cur_x_)), static_cast<int>(std::lround(cur_y_)),
            static_cast<int>(std::lround(cur_r_)));
}

void EyeMode::leave()
{
    dot_valid_ = false;
}

DisplayMode &eye_mode()
{
    static EyeMode instance;
    return instance;
}
