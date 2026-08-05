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
#include <cstdio>
#include <cstring>
#include <numbers>
#include <span>

#include "esp_log.h"
#include "esp_timer.h"

#include "colors.h"
#include "config.h"

static const char *TAG = "mode_radar";

namespace
{

constexpr uint16_t swap16(uint16_t c)
{
    return static_cast<uint16_t>((c >> 8) | (c << 8));
}

constexpr uint16_t color_green_bright = swap16(0x07E0);
constexpr uint16_t color_green_dim = swap16(0x0320);
constexpr uint16_t color_green_faint = swap16(0x0160);

inline uint16_t green_intensity(uint8_t level)
{
    uint16_t g = (static_cast<uint16_t>(level) * 63) / 255;
    uint16_t rgb565 = g << 5;
    return swap16(rgb565);
}

constexpr int font_w = 6;
constexpr int font_h = 8;
constexpr int font_first = 32;
constexpr int font_last = 126;

constexpr uint8_t font_6x8[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*   */
    0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, /* ! */
    0x00, 0x07, 0x00, 0x07, 0x00, 0x00, /* " */
    0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, /* # */
    0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, /* $ */
    0x23, 0x13, 0x08, 0x64, 0x62, 0x00, /* % */
    0x36, 0x49, 0x55, 0x22, 0x50, 0x00, /* & */
    0x00, 0x05, 0x03, 0x00, 0x00, 0x00, /* ' */
    0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, /* ( */
    0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, /* ) */
    0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, /* * */
    0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, /* + */
    0x00, 0x50, 0x30, 0x00, 0x00, 0x00, /* , */
    0x08, 0x08, 0x08, 0x08, 0x08, 0x00, /* - */
    0x00, 0x60, 0x60, 0x00, 0x00, 0x00, /* . */
    0x20, 0x10, 0x08, 0x04, 0x02, 0x00, /* / */
    0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, /* 0 */
    0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, /* 1 */
    0x42, 0x61, 0x51, 0x49, 0x46, 0x00, /* 2 */
    0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, /* 3 */
    0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, /* 4 */
    0x27, 0x45, 0x45, 0x45, 0x39, 0x00, /* 5 */
    0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, /* 6 */
    0x01, 0x71, 0x09, 0x05, 0x03, 0x00, /* 7 */
    0x36, 0x49, 0x49, 0x49, 0x36, 0x00, /* 8 */
    0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, /* 9 */
    0x00, 0x36, 0x36, 0x00, 0x00, 0x00, /* : */
    0x00, 0x56, 0x36, 0x00, 0x00, 0x00, /* ; */
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00, /* < */
    0x14, 0x14, 0x14, 0x14, 0x14, 0x00, /* = */
    0x41, 0x22, 0x14, 0x08, 0x00, 0x00, /* > */
    0x02, 0x01, 0x51, 0x09, 0x06, 0x00, /* ? */
    0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, /* @ */
    0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, /* A */
    0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, /* B */
    0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, /* C */
    0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, /* D */
    0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, /* E */
    0x7F, 0x09, 0x09, 0x01, 0x01, 0x00, /* F */
    0x3E, 0x41, 0x41, 0x51, 0x32, 0x00, /* G */
    0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, /* H */
    0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, /* I */
    0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, /* J */
    0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, /* K */
    0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, /* L */
    0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00, /* M */
    0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, /* N */
    0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, /* O */
    0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, /* P */
    0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, /* Q */
    0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, /* R */
    0x46, 0x49, 0x49, 0x49, 0x31, 0x00, /* S */
    0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, /* T */
    0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, /* U */
    0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, /* V */
    0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00, /* W */
    0x63, 0x14, 0x08, 0x14, 0x63, 0x00, /* X */
    0x03, 0x04, 0x78, 0x04, 0x03, 0x00, /* Y */
    0x61, 0x51, 0x49, 0x45, 0x43, 0x00, /* Z */
    0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, /* [ */
    0x02, 0x04, 0x08, 0x10, 0x20, 0x00, /* \ */
    0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, /* ] */
    0x04, 0x02, 0x01, 0x02, 0x04, 0x00, /* ^ */
    0x40, 0x40, 0x40, 0x40, 0x40, 0x00, /* _ */
    0x00, 0x01, 0x02, 0x04, 0x00, 0x00, /* ` */
    0x20, 0x54, 0x54, 0x54, 0x78, 0x00, /* a */
    0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, /* b */
    0x38, 0x44, 0x44, 0x44, 0x20, 0x00, /* c */
    0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, /* d */
    0x38, 0x54, 0x54, 0x54, 0x18, 0x00, /* e */
    0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, /* f */
    0x08, 0x54, 0x54, 0x54, 0x3C, 0x00, /* g */
    0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, /* h */
    0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, /* i */
    0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, /* j */
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, /* k */
    0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, /* l */
    0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, /* m */
    0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, /* n */
    0x38, 0x44, 0x44, 0x44, 0x38, 0x00, /* o */
    0x7C, 0x14, 0x14, 0x14, 0x08, 0x00, /* p */
    0x08, 0x14, 0x14, 0x18, 0x7C, 0x00, /* q */
    0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, /* r */
    0x48, 0x54, 0x54, 0x54, 0x20, 0x00, /* s */
    0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, /* t */
    0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, /* u */
    0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, /* v */
    0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, /* w */
    0x44, 0x28, 0x10, 0x28, 0x44, 0x00, /* x */
    0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00, /* y */
    0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, /* z */
    0x00, 0x08, 0x36, 0x41, 0x00, 0x00, /* { */
    0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, /* | */
    0x00, 0x41, 0x36, 0x08, 0x00, 0x00, /* } */
    0x08, 0x08, 0x2A, 0x1C, 0x08, 0x00, /* ~ */
};

constexpr float deg2rad = std::numbers::pi_v<float> / 180.0f;
constexpr int sweep_radius = 110;
constexpr int center_x = 120;
constexpr int center_y = 120;
constexpr float fov_center_deg = 90.0f;
constexpr float fov_half_deg = 60.0f;
constexpr int tail_lut_len = 24;

float angle_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

void map_target_to_screen(const Target &t, float &out_angle, float &out_norm)
{
    float az_rad = std::atan2(static_cast<float>(t.x_mm), static_cast<float>(t.y_mm));
    float az_deg = az_rad * (180.0f / std::numbers::pi_v<float>);
    out_angle = fov_center_deg + az_deg;

    float dist = static_cast<float>(t.distance_mm);
    float min_d = static_cast<float>(cfg::filter::min_dist_mm);
    float max_d = static_cast<float>(cfg::filter::max_dist_mm);
    if (max_d <= min_d)
        max_d = min_d + 1.0f;
    out_norm = (dist - min_d) / (max_d - min_d);
    if (out_norm < 0.0f)
        out_norm = 0.0f;
    if (out_norm > 1.0f)
        out_norm = 1.0f;
}

uint8_t deg_to_lut(float deg)
{
    float norm = deg / 360.0f;
    norm -= std::floor(norm);
    return static_cast<uint8_t>(norm * 255.0f);
}

uint8_t ang_diff_lut(uint8_t a, uint8_t b)
{
    int d = static_cast<int>(a) - static_cast<int>(b);
    if (d < 0)
        d += 256;
    return static_cast<uint8_t>(d);
}

} // anonymous namespace

class RadarMode : public DisplayMode
{
  public:
    const char *name() const override { return "radar"; }
    void enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
               std::span<uint16_t> scanline) override;
    void render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                const ModeFrame &frame) override;
    void leave() override;

  private:
    struct RadarTarget
    {
        bool active{};
        float angle_deg{};
        float norm_dist{};
        int64_t lit_time_us{};
        int16_t x_mm{}, y_mm{};
        uint16_t distance_mm{};
        int16_t speed_cm_s{};
    };

    esp_lcd_panel_handle_t left_{};
    esp_lcd_panel_handle_t right_{};
    std::span<uint16_t> scanline_{};
    RadarTarget targets_[mode_frame_max_targets]{};
    int64_t sweep_start_us_{};
    float last_sweep_deg_{};
    uint32_t last_render_frame_{};
    char prev_lines_[7][20]{};

    uint8_t dist_lut_[cfg::lcd::v_res][cfg::lcd::h_res]{};
    uint8_t ang_lut_[cfg::lcd::v_res][cfg::lcd::h_res]{};

    float sweep_angle_now() const;
    void build_luts();
    void update_targets(const ModeFrame &frame, float sweep_deg);
    void render_left();
    void render_right(const ModeFrame &frame);
    void draw_char(esp_lcd_panel_handle_t panel, int cx, int cy, char ch, uint16_t color,
                   uint16_t bg, int scale);
    void draw_string(esp_lcd_panel_handle_t panel, int x, int y, const char *str, uint16_t color,
                     uint16_t bg, int scale);
};

float RadarMode::sweep_angle_now() const
{
    int64_t elapsed = esp_timer_get_time() - sweep_start_us_;
    float frac = static_cast<float>(elapsed % (static_cast<int64_t>(cfg::sweep::ms) * 1000)) /
                 static_cast<float>(static_cast<int64_t>(cfg::sweep::ms) * 1000);
    return frac * 360.0f;
}

void RadarMode::build_luts()
{
    for (int y = 0; y < cfg::lcd::v_res; ++y)
    {
        for (int x = 0; x < cfg::lcd::h_res; ++x)
        {
            float dx = static_cast<float>(x - center_x);
            float dy = static_cast<float>(y - center_y);
            float d = std::sqrt(dx * dx + dy * dy);
            if (d > static_cast<float>(sweep_radius))
            {
                dist_lut_[y][x] = 0;
                ang_lut_[y][x] = 0;
            }
            else
            {
                dist_lut_[y][x] =
                    static_cast<uint8_t>((d / static_cast<float>(sweep_radius)) * 254.0f) + 1;
                float ang = std::atan2(dy, dx);
                if (ang < 0.0f)
                    ang += 2.0f * std::numbers::pi_v<float>;
                ang_lut_[y][x] =
                    static_cast<uint8_t>(ang / (2.0f * std::numbers::pi_v<float>)*255.0f);
            }
        }
    }
}

void RadarMode::update_targets(const ModeFrame &frame, float sweep_deg)
{
    const int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < mode_frame_max_targets; ++i)
    {
        if (i < frame.target_count && frame.targets[i].valid)
        {
            float ang, nd;
            map_target_to_screen(frame.targets[i], ang, nd);
            const float lo = fov_center_deg - fov_half_deg;
            const float hi = fov_center_deg + fov_half_deg;
            if (ang < lo)
                ang = lo;
            if (ang > hi)
                ang = hi;
            targets_[i].active = true;
            targets_[i].angle_deg = ang;
            targets_[i].norm_dist = nd;
            targets_[i].x_mm = frame.targets[i].x_mm;
            targets_[i].y_mm = frame.targets[i].y_mm;
            targets_[i].distance_mm = frame.targets[i].distance_mm;
            targets_[i].speed_cm_s = frame.targets[i].speed_cm_s;

            float d = angle_diff(sweep_deg, ang);
            float d_prev = angle_diff(last_sweep_deg_, ang);
            if (d >= 0.0f && d < 30.0f && d_prev < 0.0f)
            {
                targets_[i].lit_time_us = now_us;
            }
        }
        else
        {
            targets_[i].active = false;
        }
    }
}

void RadarMode::render_left()
{
    const float sweep_deg = sweep_angle_now();
    const int64_t now_us = esp_timer_get_time();
    uint8_t lut_sweep = deg_to_lut(sweep_deg);

    int tgt_x[mode_frame_max_targets], tgt_y[mode_frame_max_targets];
    for (int i = 0; i < mode_frame_max_targets; ++i)
    {
        if (!targets_[i].active)
        {
            tgt_x[i] = -100;
            tgt_y[i] = -100;
            continue;
        }
        float tang = targets_[i].angle_deg * deg2rad;
        float td = targets_[i].norm_dist * static_cast<float>(sweep_radius);
        tgt_x[i] = center_x + static_cast<int>(std::cos(tang) * td);
        tgt_y[i] = center_y + static_cast<int>(std::sin(tang) * td);
    }

    const float fov_lo_rad = (fov_center_deg - fov_half_deg) * deg2rad;
    const float fov_hi_rad = (fov_center_deg + fov_half_deg) * deg2rad;
    const float lo_dx = std::cos(fov_lo_rad), lo_dy = std::sin(fov_lo_rad);
    const float hi_dx = std::cos(fov_hi_rad), hi_dy = std::sin(fov_hi_rad);
    const float ring1 = sweep_radius / 3.0f;
    const float ring2 = sweep_radius * 2.0f / 3.0f;
    const float ring3 = static_cast<float>(sweep_radius);

    for (int y = 0; y < cfg::lcd::v_res; ++y)
    {
        for (int x = 0; x < cfg::lcd::h_res; ++x)
        {
            if (dist_lut_[y][x] == 0)
            {
                scanline_[x] = colors::black;
                continue;
            }

            uint8_t pa = ang_lut_[y][x];
            float dist =
                static_cast<float>(dist_lut_[y][x] - 1) / 254.0f * static_cast<float>(sweep_radius);
            uint16_t pix = colors::black;

            uint8_t sweep_dist = ang_diff_lut(pa, lut_sweep);
            bool on_sweep = (sweep_dist <= 1 || sweep_dist >= 254) && dist > 3.0f;

            uint8_t behind = ang_diff_lut(lut_sweep, pa);
            bool in_tail = behind > 0 && behind <= tail_lut_len && dist > 3.0f;

            if (on_sweep)
            {
                pix = color_green_bright;
            }
            else if (in_tail)
            {
                float frac =
                    1.0f - static_cast<float>(behind) / static_cast<float>(tail_lut_len + 1);
                pix = green_intensity(static_cast<uint8_t>(frac * 255.0f));
            }
            else
            {
                float px = static_cast<float>(x - center_x);
                float py = static_cast<float>(y - center_y);
                if (dist > 5.0f)
                {
                    float cross_lo = std::fabs(px * lo_dy - py * lo_dx);
                    if (cross_lo < 1.5f && (px * lo_dx + py * lo_dy) > 0.0f)
                        pix = color_green_dim;
                    float cross_hi = std::fabs(px * hi_dy - py * hi_dx);
                    if (cross_hi < 1.5f && (px * hi_dx + py * hi_dy) > 0.0f)
                        pix = color_green_dim;
                }

                if (std::fabs(dist - ring1) < 1.2f || std::fabs(dist - ring2) < 1.2f ||
                    std::fabs(dist - ring3) < 1.2f)
                {
                    if (pix == colors::black)
                        pix = color_green_faint;
                }
            }

            scanline_[x] = pix;
        }

        for (int t = 0; t < mode_frame_max_targets; ++t)
        {
            if (!targets_[t].active)
                continue;
            int dty = y - tgt_y[t];
            if (dty < -4 || dty > 4)
                continue;
            for (int dtx = -4; dtx <= 4; ++dtx)
            {
                int bx = tgt_x[t] + dtx;
                if (bx < 0 || bx >= cfg::lcd::h_res)
                    continue;
                if (dtx * dtx + dty * dty <= 16)
                {
                    int64_t age = now_us - targets_[t].lit_time_us;
                    float age_frac =
                        static_cast<float>(age) /
                        static_cast<float>(static_cast<int64_t>(cfg::sweep::ms) * 1000);
                    if (age_frac > 1.0f)
                        age_frac = 1.0f;
                    uint8_t bright = static_cast<uint8_t>(255.0f * (1.0f - age_frac * 0.85f));
                    uint16_t c = green_intensity(bright);
                    if (c > scanline_[bx])
                        scanline_[bx] = c;
                }
            }
        }

        esp_lcd_panel_draw_bitmap(left_, 0, y, cfg::lcd::h_res, y + 1, scanline_.data());
    }

    last_sweep_deg_ = sweep_deg;
}

void RadarMode::draw_char(esp_lcd_panel_handle_t panel, int cx, int cy, char ch, uint16_t color,
                          uint16_t bg, int scale)
{
    if (ch < font_first || ch > font_last)
        ch = ' ';
    const uint8_t *glyph = &font_6x8[(ch - font_first) * font_w];
    const int pw = font_w * scale;

    for (int r = 0; r < font_h; ++r)
    {
        for (int c = 0; c < font_w; ++c)
        {
            uint16_t pix = (glyph[c] & (1 << r)) ? color : bg;
            for (int s = 0; s < scale; ++s)
                scanline_[c * scale + s] = pix;
        }
        int py = cy + r * scale;
        for (int s = 0; s < scale; ++s)
        {
            if (py + s >= 0 && py + s < cfg::lcd::v_res)
            {
                int ex = cx + pw;
                if (ex > cfg::lcd::h_res)
                    ex = cfg::lcd::h_res;
                if (cx >= 0 && cx < cfg::lcd::h_res)
                    esp_lcd_panel_draw_bitmap(panel, cx, py + s, ex, py + s + 1, scanline_.data());
            }
        }
    }
}

void RadarMode::draw_string(esp_lcd_panel_handle_t panel, int x, int y, const char *str,
                            uint16_t color, uint16_t bg, int scale)
{
    while (*str)
    {
        draw_char(panel, x, y, *str, color, bg, scale);
        x += font_w * scale;
        str++;
    }
}

void RadarMode::render_right(const ModeFrame &frame)
{
    const int scale = 2;
    const int char_w = font_w * scale;
    const int line_h = font_h * scale + 4;
    const int max_chars = 14;
    const int num_lines = 7;
    const int block_w = max_chars * char_w;
    const int block_h = num_lines * line_h;
    const int x0 = (cfg::lcd::h_res - block_w) / 2;
    const int y0 = (cfg::lcd::v_res - block_h) / 2;

    char lines[7][20];
    memset(lines, ' ', sizeof(lines));

    if (frame.human && frame.primary_idx >= 0)
    {
        const Target *t = &frame.primary;
        snprintf(lines[0], 19, "TGT %d/%d", frame.primary_idx + 1, frame.target_count);
        snprintf(lines[1], 19, "--------");
        snprintf(lines[2], 19, "X:  %5d mm", static_cast<int>(t->x_mm));
        snprintf(lines[3], 19, "Y:  %5d mm", static_cast<int>(t->y_mm));
        snprintf(lines[4], 19, "DST: %4u mm", static_cast<unsigned>(t->distance_mm));
        snprintf(lines[5], 19, "SPD: %4d cm/s", static_cast<int>(t->speed_cm_s));
        snprintf(lines[6], 19, "AZ:  %5.1f", static_cast<double>(frame.azimuth_deg));
    }
    else
    {
        snprintf(lines[3], 19, "  NO TARGET");
    }

    for (int i = 0; i < 7; ++i)
    {
        for (int j = 0; j < max_chars; ++j)
            if (lines[i][j] == '\0')
                lines[i][j] = ' ';
        lines[i][max_chars] = '\0';
    }

    for (int i = 0; i < 7; ++i)
    {
        if (memcmp(lines[i], prev_lines_[i], max_chars + 1) == 0)
            continue;
        memcpy(prev_lines_[i], lines[i], max_chars + 1);
        draw_string(right_, x0, y0 + i * line_h, lines[i], color_green_bright, colors::black,
                    scale);
    }
}

void RadarMode::enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                      std::span<uint16_t> scanline)
{
    left_ = left;
    right_ = right;
    scanline_ = scanline;
    sweep_start_us_ = esp_timer_get_time();
    last_sweep_deg_ = 0.0f;
    last_render_frame_ = 0;
    for (auto &t : targets_)
        t = {};
    memset(prev_lines_, 0, sizeof(prev_lines_));

    esp_lcd_panel_swap_xy(left_, true);
    esp_lcd_panel_mirror(left_, true, true);

    esp_lcd_panel_swap_xy(right_, true);
    esp_lcd_panel_mirror(right_, true, true);

    build_luts();
    fill_panel(left_, colors::black, scanline_);
    fill_panel(right_, colors::black, scanline_);

    ESP_LOGI(TAG, "entered radar mode");
}

void RadarMode::render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                       const ModeFrame &frame)
{
    (void)left;
    (void)right;
    if (frame.frame_id == last_render_frame_)
        return;
    last_render_frame_ = frame.frame_id;

    update_targets(frame, sweep_angle_now());
    render_left();
    render_right(frame);
}

void RadarMode::leave()
{
    esp_lcd_panel_swap_xy(left_, true);
    esp_lcd_panel_mirror(left_, true, false);

    esp_lcd_panel_swap_xy(right_, true);
    esp_lcd_panel_mirror(right_, false, true);
}

DisplayMode &radar_mode()
{
    static RadarMode instance;
    return instance;
}
