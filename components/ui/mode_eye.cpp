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

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "mode_eye";

#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000

static esp_lcd_panel_handle_t s_left;
static esp_lcd_panel_handle_t s_right;

static int s_dot_x = -1;
static int s_dot_y = -1;
static int s_dot_r = 0;
static bool s_dot_valid;

static float s_cur_x, s_cur_y, s_cur_r;

static void fill_panel(esp_lcd_panel_handle_t panel, uint16_t color)
{
    const size_t line_bytes = CFG_LCD_H_RES * sizeof(uint16_t);
    uint16_t *line = static_cast<uint16_t *>(heap_caps_malloc(line_bytes, MALLOC_CAP_DMA));
    if (!line)
        return;
    for (int x = 0; x < CFG_LCD_H_RES; ++x)
        line[x] = color;
    for (int y = 0; y < CFG_LCD_V_RES; ++y)
        esp_lcd_panel_draw_bitmap(panel, 0, y, CFG_LCD_H_RES, y + 1, line);
    free(line);
}

static void move_dot_on_panel(esp_lcd_panel_handle_t panel, int ox, int oy, int orad, int nx,
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
    if (bx1 >= CFG_LCD_H_RES)
        bx1 = CFG_LCD_H_RES - 1;
    if (by0 < 0)
        by0 = 0;
    if (by1 >= CFG_LCD_V_RES)
        by1 = CFG_LCD_V_RES - 1;

    const int w = bx1 - bx0 + 1;
    uint16_t *row =
        static_cast<uint16_t *>(heap_caps_malloc((size_t)w * sizeof(uint16_t), MALLOC_CAP_DMA));
    if (!row)
        return;

    for (int y = by0; y <= by1; ++y)
    {
        for (int i = 0; i < w; ++i)
        {
            int x = bx0 + i;
            int dx = x - nx, dy = y - ny;
            row[i] = (dx * dx + dy * dy <= nr2) ? COLOR_BLACK : COLOR_WHITE;
        }
        esp_lcd_panel_draw_bitmap(panel, bx0, y, bx1 + 1, y + 1, row);
    }
    free(row);
}

static void draw_filled_circle(esp_lcd_panel_handle_t panel, int cx, int cy, int radius,
                               uint16_t color)
{
    const int r2 = radius * radius;
    const int x0 = cx - radius;
    const int x1 = cx + radius;
    const int y0 = cy - radius;
    const int y1 = cy + radius;
    const int w = x1 - x0 + 1;

    uint16_t *row =
        static_cast<uint16_t *>(heap_caps_malloc((size_t)w * sizeof(uint16_t), MALLOC_CAP_DMA));
    if (!row)
        return;

    for (int y = y0; y <= y1; ++y)
    {
        if (y < 0 || y >= CFG_LCD_V_RES)
            continue;
        int draw_x0 = -1, draw_x1 = -1;
        for (int x = x0; x <= x1; ++x)
        {
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r2 && x >= 0 && x < CFG_LCD_H_RES)
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
            row[i] = color;
        esp_lcd_panel_draw_bitmap(panel, draw_x0, y, draw_x1 + 1, y + 1, row);
    }
    free(row);
}

static void set_dot(int x, int y, int radius)
{
    if (radius < 1)
        radius = 1;
    if (radius > CFG_DOT_RADIUS_MAX)
        radius = CFG_DOT_RADIUS_MAX;
    if (x < radius)
        x = radius;
    else if (x >= CFG_LCD_H_RES - radius)
        x = CFG_LCD_H_RES - radius - 1;
    if (y < radius)
        y = radius;
    else if (y >= CFG_LCD_V_RES - radius)
        y = CFG_LCD_V_RES - radius - 1;

    if (s_dot_valid && s_dot_x == x && s_dot_y == y && s_dot_r == radius)
        return;

    if (s_dot_valid)
    {
        move_dot_on_panel(s_left, s_dot_x, s_dot_y, s_dot_r, x, y, radius);
        move_dot_on_panel(s_right, s_dot_x, s_dot_y, s_dot_r, x, y, radius);
    }
    else
    {
        draw_filled_circle(s_left, x, y, radius, COLOR_BLACK);
        draw_filled_circle(s_right, x, y, radius, COLOR_BLACK);
    }
    s_dot_x = x;
    s_dot_y = y;
    s_dot_r = radius;
    s_dot_valid = true;
}

static void clamp_to_circle(float *x, float *y)
{
    const float cx = (CFG_LCD_H_RES - 1) * 0.5f;
    const float cy = (CFG_LCD_V_RES - 1) * 0.5f;
    const float max_r = (CFG_LCD_H_RES * 0.5f) - (float)CFG_DOT_RADIUS_MAX - 2.0f;
    const float dx = *x - cx;
    const float dy = *y - cy;
    const float dist = sqrtf(dx * dx + dy * dy);
    if (dist > max_r && dist > 0.0f)
    {
        const float s = max_r / dist;
        *x = cx + dx * s;
        *y = cy + dy * s;
    }
}

static float radius_from_distance(uint16_t dist_mm)
{
    float d = (float)dist_mm;
    if (d <= (float)CFG_DOT_NEAR_MM)
        return (float)CFG_DOT_RADIUS_MAX;
    if (d >= (float)CFG_DOT_FAR_MM)
        return (float)CFG_DOT_RADIUS_MIN;
    float t = (d - (float)CFG_DOT_NEAR_MM) / (float)(CFG_DOT_FAR_MM - CFG_DOT_NEAR_MM);
    return (float)CFG_DOT_RADIUS_MAX + t * (float)(CFG_DOT_RADIUS_MIN - CFG_DOT_RADIUS_MAX);
}

static void gaze_xy(float az_deg, float elev_norm, float *tx, float *ty)
{
    float deg = az_deg;
    if (deg < -CFG_GAZE_MAX_DEG)
        deg = -CFG_GAZE_MAX_DEG;
    else if (deg > CFG_GAZE_MAX_DEG)
        deg = CFG_GAZE_MAX_DEG;

    *tx = ((deg + CFG_GAZE_MAX_DEG) / (2.0f * CFG_GAZE_MAX_DEG)) * (float)(CFG_LCD_H_RES - 1);

    float elev = elev_norm;
    if (elev < 0.0f)
        elev = 0.0f;
    else if (elev > 1.0f)
        elev = 1.0f;
    *ty = (1.0f - elev) * (float)(CFG_LCD_V_RES - 1);
    clamp_to_circle(tx, ty);
}

/* ── mode interface ──────────────────────────────────────────────── */

static void eye_enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right)
{
    s_left = left;
    s_right = right;
    s_dot_valid = false;
    s_dot_x = -1;
    s_dot_y = -1;
    s_dot_r = 0;
    s_cur_x = (float)(CFG_LCD_H_RES / 2);
    s_cur_y = (float)(CFG_LCD_V_RES / 2);
    s_cur_r = (float)CFG_DOT_RADIUS_MIN;

    fill_panel(s_left, COLOR_WHITE);
    fill_panel(s_right, COLOR_WHITE);

    set_dot((int)s_cur_x, (int)s_cur_y, (int)s_cur_r);
    ESP_LOGI(TAG, "entered eye mode");
}

static void eye_render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                       const mode_frame_t *frame)
{
    (void)left;
    (void)right;

    float tgt_x, tgt_y, tgt_r;
    if (frame->human)
    {
        gaze_xy(frame->azimuth_deg, frame->elevation_norm, &tgt_x, &tgt_y);
        tgt_r = radius_from_distance(frame->primary.distance_mm);
    }
    else
    {
        tgt_x = (float)(CFG_LCD_H_RES / 2);
        tgt_y = (float)(CFG_LCD_V_RES / 2);
        tgt_r = (float)CFG_DOT_RADIUS_MIN;
    }

    s_cur_x += (tgt_x - s_cur_x) * CFG_SMOOTH_H;
    s_cur_y += (tgt_y - s_cur_y) * CFG_SMOOTH_V;
    s_cur_r += (tgt_r - s_cur_r) * CFG_SMOOTH_R;

    set_dot((int)lroundf(s_cur_x), (int)lroundf(s_cur_y), (int)lroundf(s_cur_r));
}

static void eye_leave(void)
{
    s_dot_valid = false;
}

const display_mode_t mode_eye = {
    .name = "eye",
    .enter = eye_enter,
    .render = eye_render,
    .leave = eye_leave,
};
