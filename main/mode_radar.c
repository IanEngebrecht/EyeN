#include "mode.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "display.h"

static const char *TAG = "mode_radar";

/* ── Colors (RGB565, byte-swapped for little-endian SPI) ─────────── */

#define COLOR_BLACK   0x0000
#define SWAP16(c) ((uint16_t)(((c) >> 8) | ((c) << 8)))
#define COLOR_GREEN_BRIGHT  SWAP16(0x07E0)
#define COLOR_GREEN_DIM     SWAP16(0x0320)
#define COLOR_GREEN_FAINT   SWAP16(0x0160)

static inline uint16_t green_intensity(uint8_t level)
{
    uint16_t g = ((uint16_t)level * 63) / 255;
    uint16_t rgb565 = (g << 5);
    return SWAP16(rgb565);
}

/* ── Embedded 6x8 bitmap font (ASCII 32–126) ─────────────────────── */

#define FONT_W 6
#define FONT_H 8
#define FONT_FIRST 32
#define FONT_LAST  126

static const uint8_t font_6x8[] = {
    /* Each char is 6 bytes; each byte is a column, LSB=top row.
       Classic 5x7 in a 6x8 cell (6th col always 0). */
    0x00,0x00,0x00,0x00,0x00,0x00, /*   */
    0x00,0x00,0x5F,0x00,0x00,0x00, /* ! */
    0x00,0x07,0x00,0x07,0x00,0x00, /* " */
    0x14,0x7F,0x14,0x7F,0x14,0x00, /* # */
    0x24,0x2A,0x7F,0x2A,0x12,0x00, /* $ */
    0x23,0x13,0x08,0x64,0x62,0x00, /* % */
    0x36,0x49,0x55,0x22,0x50,0x00, /* & */
    0x00,0x05,0x03,0x00,0x00,0x00, /* ' */
    0x00,0x1C,0x22,0x41,0x00,0x00, /* ( */
    0x00,0x41,0x22,0x1C,0x00,0x00, /* ) */
    0x08,0x2A,0x1C,0x2A,0x08,0x00, /* * */
    0x08,0x08,0x3E,0x08,0x08,0x00, /* + */
    0x00,0x50,0x30,0x00,0x00,0x00, /* , */
    0x08,0x08,0x08,0x08,0x08,0x00, /* - */
    0x00,0x60,0x60,0x00,0x00,0x00, /* . */
    0x20,0x10,0x08,0x04,0x02,0x00, /* / */
    0x3E,0x51,0x49,0x45,0x3E,0x00, /* 0 */
    0x00,0x42,0x7F,0x40,0x00,0x00, /* 1 */
    0x42,0x61,0x51,0x49,0x46,0x00, /* 2 */
    0x21,0x41,0x45,0x4B,0x31,0x00, /* 3 */
    0x18,0x14,0x12,0x7F,0x10,0x00, /* 4 */
    0x27,0x45,0x45,0x45,0x39,0x00, /* 5 */
    0x3C,0x4A,0x49,0x49,0x30,0x00, /* 6 */
    0x01,0x71,0x09,0x05,0x03,0x00, /* 7 */
    0x36,0x49,0x49,0x49,0x36,0x00, /* 8 */
    0x06,0x49,0x49,0x29,0x1E,0x00, /* 9 */
    0x00,0x36,0x36,0x00,0x00,0x00, /* : */
    0x00,0x56,0x36,0x00,0x00,0x00, /* ; */
    0x00,0x08,0x14,0x22,0x41,0x00, /* < */
    0x14,0x14,0x14,0x14,0x14,0x00, /* = */
    0x41,0x22,0x14,0x08,0x00,0x00, /* > */
    0x02,0x01,0x51,0x09,0x06,0x00, /* ? */
    0x32,0x49,0x79,0x41,0x3E,0x00, /* @ */
    0x7E,0x11,0x11,0x11,0x7E,0x00, /* A */
    0x7F,0x49,0x49,0x49,0x36,0x00, /* B */
    0x3E,0x41,0x41,0x41,0x22,0x00, /* C */
    0x7F,0x41,0x41,0x22,0x1C,0x00, /* D */
    0x7F,0x49,0x49,0x49,0x41,0x00, /* E */
    0x7F,0x09,0x09,0x01,0x01,0x00, /* F */
    0x3E,0x41,0x41,0x51,0x32,0x00, /* G */
    0x7F,0x08,0x08,0x08,0x7F,0x00, /* H */
    0x00,0x41,0x7F,0x41,0x00,0x00, /* I */
    0x20,0x40,0x41,0x3F,0x01,0x00, /* J */
    0x7F,0x08,0x14,0x22,0x41,0x00, /* K */
    0x7F,0x40,0x40,0x40,0x40,0x00, /* L */
    0x7F,0x02,0x04,0x02,0x7F,0x00, /* M */
    0x7F,0x04,0x08,0x10,0x7F,0x00, /* N */
    0x3E,0x41,0x41,0x41,0x3E,0x00, /* O */
    0x7F,0x09,0x09,0x09,0x06,0x00, /* P */
    0x3E,0x41,0x51,0x21,0x5E,0x00, /* Q */
    0x7F,0x09,0x19,0x29,0x46,0x00, /* R */
    0x46,0x49,0x49,0x49,0x31,0x00, /* S */
    0x01,0x01,0x7F,0x01,0x01,0x00, /* T */
    0x3F,0x40,0x40,0x40,0x3F,0x00, /* U */
    0x1F,0x20,0x40,0x20,0x1F,0x00, /* V */
    0x7F,0x20,0x18,0x20,0x7F,0x00, /* W */
    0x63,0x14,0x08,0x14,0x63,0x00, /* X */
    0x03,0x04,0x78,0x04,0x03,0x00, /* Y */
    0x61,0x51,0x49,0x45,0x43,0x00, /* Z */
    0x00,0x00,0x7F,0x41,0x41,0x00, /* [ */
    0x02,0x04,0x08,0x10,0x20,0x00, /* \ */
    0x41,0x41,0x7F,0x00,0x00,0x00, /* ] */
    0x04,0x02,0x01,0x02,0x04,0x00, /* ^ */
    0x40,0x40,0x40,0x40,0x40,0x00, /* _ */
    0x00,0x01,0x02,0x04,0x00,0x00, /* ` */
    0x20,0x54,0x54,0x54,0x78,0x00, /* a */
    0x7F,0x48,0x44,0x44,0x38,0x00, /* b */
    0x38,0x44,0x44,0x44,0x20,0x00, /* c */
    0x38,0x44,0x44,0x48,0x7F,0x00, /* d */
    0x38,0x54,0x54,0x54,0x18,0x00, /* e */
    0x08,0x7E,0x09,0x01,0x02,0x00, /* f */
    0x08,0x54,0x54,0x54,0x3C,0x00, /* g */
    0x7F,0x08,0x04,0x04,0x78,0x00, /* h */
    0x00,0x44,0x7D,0x40,0x00,0x00, /* i */
    0x20,0x40,0x44,0x3D,0x00,0x00, /* j */
    0x00,0x7F,0x10,0x28,0x44,0x00, /* k */
    0x00,0x41,0x7F,0x40,0x00,0x00, /* l */
    0x7C,0x04,0x18,0x04,0x78,0x00, /* m */
    0x7C,0x08,0x04,0x04,0x78,0x00, /* n */
    0x38,0x44,0x44,0x44,0x38,0x00, /* o */
    0x7C,0x14,0x14,0x14,0x08,0x00, /* p */
    0x08,0x14,0x14,0x18,0x7C,0x00, /* q */
    0x7C,0x08,0x04,0x04,0x08,0x00, /* r */
    0x48,0x54,0x54,0x54,0x20,0x00, /* s */
    0x04,0x3F,0x44,0x40,0x20,0x00, /* t */
    0x3C,0x40,0x40,0x20,0x7C,0x00, /* u */
    0x1C,0x20,0x40,0x20,0x1C,0x00, /* v */
    0x3C,0x40,0x30,0x40,0x3C,0x00, /* w */
    0x44,0x28,0x10,0x28,0x44,0x00, /* x */
    0x0C,0x50,0x50,0x50,0x3C,0x00, /* y */
    0x44,0x64,0x54,0x4C,0x44,0x00, /* z */
    0x00,0x08,0x36,0x41,0x00,0x00, /* { */
    0x00,0x00,0x7F,0x00,0x00,0x00, /* | */
    0x00,0x41,0x36,0x08,0x00,0x00, /* } */
    0x08,0x08,0x2A,0x1C,0x08,0x00, /* ~ */
};

static void draw_char(esp_lcd_panel_handle_t panel, int cx, int cy,
                      char ch, uint16_t color, uint16_t bg, int scale)
{
    if (ch < FONT_FIRST || ch > FONT_LAST) ch = ' ';
    const uint8_t *glyph = &font_6x8[(ch - FONT_FIRST) * FONT_W];
    const int pw = FONT_W * scale;

    uint16_t *row = heap_caps_malloc((size_t)pw * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!row) return;

    for (int r = 0; r < FONT_H; ++r) {
        for (int c = 0; c < FONT_W; ++c) {
            uint16_t pix = (glyph[c] & (1 << r)) ? color : bg;
            for (int s = 0; s < scale; ++s)
                row[c * scale + s] = pix;
        }
        int py = cy + r * scale;
        for (int s = 0; s < scale; ++s) {
            if (py + s >= 0 && py + s < CFG_LCD_V_RES) {
                int ex = cx + pw;
                if (ex > CFG_LCD_H_RES) ex = CFG_LCD_H_RES;
                if (cx >= 0 && cx < CFG_LCD_H_RES)
                    esp_lcd_panel_draw_bitmap(panel, cx, py + s, ex, py + s + 1, row);
            }
        }
    }
    free(row);
}

static void draw_string(esp_lcd_panel_handle_t panel, int x, int y,
                        const char *str, uint16_t color, uint16_t bg, int scale)
{
    while (*str) {
        draw_char(panel, x, y, *str, color, bg, scale);
        x += FONT_W * scale;
        str++;
    }
}

/* ── Radar sweep state ───────────────────────────────────────────── */

#define PI_F 3.14159265f
#define DEG2RAD (PI_F / 180.0f)

#define SWEEP_RADIUS  110
#define CENTER_X      120
#define CENTER_Y      120

/* FOV wedge: 120° centered pointing down (90° in screen coords = +Y) */
#define FOV_CENTER_DEG  90.0f
#define FOV_HALF_DEG    60.0f

/* Sweep tail length in 0‥255 LUT angle units (~34°) */
#define TAIL_LUT_LEN    24

static esp_lcd_panel_handle_t s_left;
static esp_lcd_panel_handle_t s_right;

typedef struct {
    bool    active;
    float   angle_deg;   /* screen angle of target */
    float   norm_dist;   /* 0‥1 normalized distance */
    int64_t lit_time_us; /* when sweep last passed */
    int16_t x_mm, y_mm;
    uint16_t distance_mm;
    int16_t speed_cm_s;
} radar_target_t;

static radar_target_t s_targets[MODE_MAX_TARGETS];
static int64_t s_sweep_start_us;
static float   s_last_sweep_deg;
static uint32_t s_last_render_frame;

static float sweep_angle_now(void)
{
    int64_t elapsed = esp_timer_get_time() - s_sweep_start_us;
    float frac = (float)(elapsed % ((int64_t)CFG_RADAR_SWEEP_MS * 1000))
                 / (float)((int64_t)CFG_RADAR_SWEEP_MS * 1000);
    return frac * 360.0f;
}

static float angle_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static void map_target_to_screen(const ld2450_target_t *t, float *out_angle, float *out_norm)
{
    /* LD2450 coords: x_mm (lateral, + right), y_mm (forward, + away).
       Screen: 270° is straight down (away from sensor). Looking down at the
       scene, targets further away map toward the rim, closer toward center. */
    float az_rad = atan2f((float)t->x_mm, (float)t->y_mm);
    float az_deg = az_rad * (180.0f / PI_F);
    *out_angle = FOV_CENTER_DEG + az_deg;

    float dist = (float)t->distance_mm;
    float min_d = (float)CFG_FILTER_MIN_DIST_MM;
    float max_d = (float)CFG_FILTER_MAX_DIST_MM;
    if (max_d <= min_d) max_d = min_d + 1.0f;
    *out_norm = (dist - min_d) / (max_d - min_d);
    if (*out_norm < 0.0f) *out_norm = 0.0f;
    if (*out_norm > 1.0f) *out_norm = 1.0f;
}

/* ── Left eye: PPI render (partial update) ───────────────────────── */

/* Pre-computed pixel screen position → polar coords (avoids per-frame trig) */
static uint8_t s_dist_lut[CFG_LCD_V_RES][CFG_LCD_H_RES];  /* distance 0‥SWEEP_RADIUS scaled to 0‥255, 0 if outside */
static uint8_t s_ang_lut[CFG_LCD_V_RES][CFG_LCD_H_RES];   /* angle 0‥255 mapping 0‥360° */

static void build_luts(void)
{
    for (int y = 0; y < CFG_LCD_V_RES; ++y) {
        for (int x = 0; x < CFG_LCD_H_RES; ++x) {
            float dx = (float)(x - CENTER_X);
            float dy = (float)(y - CENTER_Y);
            float d = sqrtf(dx * dx + dy * dy);
            if (d > (float)SWEEP_RADIUS) {
                s_dist_lut[y][x] = 0;
                s_ang_lut[y][x] = 0;
            } else {
                s_dist_lut[y][x] = (uint8_t)((d / (float)SWEEP_RADIUS) * 254.0f) + 1;
                float ang = atan2f(dy, dx);
                if (ang < 0.0f) ang += 2.0f * PI_F;
                s_ang_lut[y][x] = (uint8_t)(ang / (2.0f * PI_F) * 255.0f);
            }
        }
    }
}

static void draw_static_grid(void)
{
    uint16_t *row = heap_caps_malloc(CFG_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!row) return;

    /* FOV boundary directions (same cos/sin space as dot placement) */
    const float fov_lo_rad = (FOV_CENTER_DEG - FOV_HALF_DEG) * DEG2RAD;
    const float fov_hi_rad = (FOV_CENTER_DEG + FOV_HALF_DEG) * DEG2RAD;
    const float lo_dx = cosf(fov_lo_rad), lo_dy = sinf(fov_lo_rad);
    const float hi_dx = cosf(fov_hi_rad), hi_dy = sinf(fov_hi_rad);
    const float ring1 = SWEEP_RADIUS / 3.0f;
    const float ring2 = SWEEP_RADIUS * 2.0f / 3.0f;
    const float ring3 = (float)SWEEP_RADIUS;

    for (int y = 0; y < CFG_LCD_V_RES; ++y) {
        for (int x = 0; x < CFG_LCD_H_RES; ++x) {
            uint16_t pix = COLOR_BLACK;
            if (s_dist_lut[y][x] > 0) {
                float px = (float)(x - CENTER_X);
                float py = (float)(y - CENTER_Y);
                float dist = (float)(s_dist_lut[y][x] - 1) / 254.0f * (float)SWEEP_RADIUS;

                /* FOV boundary lines: distance from pixel to each boundary ray */
                if (dist > 5.0f) {
                    float cross_lo = fabsf(px * lo_dy - py * lo_dx);
                    float dot_lo = px * lo_dx + py * lo_dy;
                    if (cross_lo < 1.5f && dot_lo > 0.0f)
                        pix = COLOR_GREEN_DIM;

                    float cross_hi = fabsf(px * hi_dy - py * hi_dx);
                    float dot_hi = px * hi_dx + py * hi_dy;
                    if (cross_hi < 1.5f && dot_hi > 0.0f)
                        pix = COLOR_GREEN_DIM;
                }

                /* Range rings */
                if (fabsf(dist - ring1) < 1.2f || fabsf(dist - ring2) < 1.2f ||
                    fabsf(dist - ring3) < 1.2f) {
                    if (pix == COLOR_BLACK) pix = COLOR_GREEN_FAINT;
                }
            }
            row[x] = pix;
        }
        esp_lcd_panel_draw_bitmap(s_left, 0, y, CFG_LCD_H_RES, y + 1, row);
    }
    free(row);
}

/* Convert degrees to the 0‥255 angle LUT space */
static inline uint8_t deg_to_lut(float deg)
{
    float norm = deg / 360.0f;
    norm -= floorf(norm);
    return (uint8_t)(norm * 255.0f);
}

static inline uint8_t ang_diff_lut(uint8_t a, uint8_t b)
{
    int d = (int)a - (int)b;
    if (d < 0) d += 256;
    return (uint8_t)d;
}

/* Update target state from a new radar frame */
static void update_targets(const mode_frame_t *frame, float sweep_deg)
{
    const int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < MODE_MAX_TARGETS; ++i) {
        if (i < frame->target_count && frame->targets[i].valid) {
            float ang, nd;
            map_target_to_screen(&frame->targets[i], &ang, &nd);
            /* Hard clamp: a target dot must never render outside the
               120° FOV wedge, even if noisy azimuth briefly exceeds it. */
            const float lo = FOV_CENTER_DEG - FOV_HALF_DEG;
            const float hi = FOV_CENTER_DEG + FOV_HALF_DEG;
            if (ang < lo) ang = lo;
            if (ang > hi) ang = hi;
            s_targets[i].active = true;
            s_targets[i].angle_deg = ang;
            s_targets[i].norm_dist = nd;
            s_targets[i].x_mm = frame->targets[i].x_mm;
            s_targets[i].y_mm = frame->targets[i].y_mm;
            s_targets[i].distance_mm = frame->targets[i].distance_mm;
            s_targets[i].speed_cm_s = frame->targets[i].speed_cm_s;

            float d = angle_diff(sweep_deg, ang);
            float d_prev = angle_diff(s_last_sweep_deg, ang);
            if (d >= 0.0f && d < 30.0f && d_prev < 0.0f) {
                s_targets[i].lit_time_us = now_us;
            }
        } else {
            s_targets[i].active = false;
        }
    }
}

/* Full-circle render: recompute every pixel each frame.
   The LUTs keep per-pixel cost to simple integer ops. */
static void render_left(void)
{
    const float sweep_deg = sweep_angle_now();
    const int64_t now_us = esp_timer_get_time();
    uint8_t lut_sweep = deg_to_lut(sweep_deg);

    /* Precompute target screen positions */
    int tgt_x[MODE_MAX_TARGETS], tgt_y[MODE_MAX_TARGETS];
    for (int i = 0; i < MODE_MAX_TARGETS; ++i) {
        if (!s_targets[i].active) { tgt_x[i] = -100; tgt_y[i] = -100; continue; }
        float tang = s_targets[i].angle_deg * DEG2RAD;
        float td   = s_targets[i].norm_dist * (float)SWEEP_RADIUS;
        tgt_x[i] = CENTER_X + (int)(cosf(tang) * td);
        tgt_y[i] = CENTER_Y + (int)(sinf(tang) * td);
    }

    uint16_t *row = heap_caps_malloc(CFG_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!row) return;

    const float fov_lo_rad = (FOV_CENTER_DEG - FOV_HALF_DEG) * DEG2RAD;
    const float fov_hi_rad = (FOV_CENTER_DEG + FOV_HALF_DEG) * DEG2RAD;
    const float lo_dx = cosf(fov_lo_rad), lo_dy = sinf(fov_lo_rad);
    const float hi_dx = cosf(fov_hi_rad), hi_dy = sinf(fov_hi_rad);
    const float ring1 = SWEEP_RADIUS / 3.0f;
    const float ring2 = SWEEP_RADIUS * 2.0f / 3.0f;
    const float ring3 = (float)SWEEP_RADIUS;

    for (int y = 0; y < CFG_LCD_V_RES; ++y) {
        for (int x = 0; x < CFG_LCD_H_RES; ++x) {
            if (s_dist_lut[y][x] == 0) { row[x] = COLOR_BLACK; continue; }

            uint8_t pa  = s_ang_lut[y][x];
            float dist  = (float)(s_dist_lut[y][x] - 1) / 254.0f * (float)SWEEP_RADIUS;
            uint16_t pix = COLOR_BLACK;

            /* Sweep line (thin) and tail glow take priority over the grid,
               so the gradient reads clean with no grid lines poking through. */
            uint8_t sweep_dist = ang_diff_lut(pa, lut_sweep);
            bool on_sweep = (sweep_dist <= 1 || sweep_dist >= 254) && dist > 3.0f;

            uint8_t behind = ang_diff_lut(lut_sweep, pa);
            bool in_tail = behind > 0 && behind <= TAIL_LUT_LEN && dist > 3.0f;

            if (on_sweep) {
                pix = COLOR_GREEN_BRIGHT;
            } else if (in_tail) {
                float frac = 1.0f - (float)behind / (float)(TAIL_LUT_LEN + 1);
                pix = green_intensity((uint8_t)(frac * 255.0f));
            } else {
                /* Grid: FOV boundary lines */
                float px = (float)(x - CENTER_X);
                float py = (float)(y - CENTER_Y);
                if (dist > 5.0f) {
                    float cross_lo = fabsf(px * lo_dy - py * lo_dx);
                    if (cross_lo < 1.5f && (px * lo_dx + py * lo_dy) > 0.0f)
                        pix = COLOR_GREEN_DIM;
                    float cross_hi = fabsf(px * hi_dy - py * hi_dx);
                    if (cross_hi < 1.5f && (px * hi_dx + py * hi_dy) > 0.0f)
                        pix = COLOR_GREEN_DIM;
                }

                /* Range rings */
                if (fabsf(dist - ring1) < 1.2f || fabsf(dist - ring2) < 1.2f ||
                    fabsf(dist - ring3) < 1.2f) {
                    if (pix == COLOR_BLACK) pix = COLOR_GREEN_FAINT;
                }
            }

            row[x] = pix;
        }

        /* Overlay target dots */
        for (int t = 0; t < MODE_MAX_TARGETS; ++t) {
            if (!s_targets[t].active) continue;
            int dty = y - tgt_y[t];
            if (dty < -4 || dty > 4) continue;
            for (int dtx = -4; dtx <= 4; ++dtx) {
                int bx = tgt_x[t] + dtx;
                if (bx < 0 || bx >= CFG_LCD_H_RES) continue;
                if (dtx * dtx + dty * dty <= 16) {
                    int64_t age = now_us - s_targets[t].lit_time_us;
                    float age_frac = (float)age / (float)((int64_t)CFG_RADAR_SWEEP_MS * 1000);
                    if (age_frac > 1.0f) age_frac = 1.0f;
                    uint8_t bright = (uint8_t)(255.0f * (1.0f - age_frac * 0.85f));
                    uint16_t c = green_intensity(bright);
                    if (c > row[bx]) row[bx] = c;
                }
            }
        }

        esp_lcd_panel_draw_bitmap(s_left, 0, y, CFG_LCD_H_RES, y + 1, row);
    }

    s_last_sweep_deg = sweep_deg;
    free(row);
}

/* ── Right eye: target data readout (partial update) ─────────────── */

static void render_right(const mode_frame_t *frame)
{
    const int scale = 2;
    const int char_w = FONT_W * scale;
    const int line_h = FONT_H * scale + 4;
    const int max_chars = 14;
    const int num_lines = 7;
    const int block_w = max_chars * char_w;
    const int block_h = num_lines * line_h;
    const int x0 = (CFG_LCD_H_RES - block_w) / 2;
    const int y0 = (CFG_LCD_V_RES - block_h) / 2;

    char lines[7][20];
    memset(lines, ' ', sizeof(lines));

    if (frame->human && frame->primary_idx >= 0) {
        const ld2450_target_t *t = &frame->primary;
        snprintf(lines[0], 19, "TGT %d/%d", frame->primary_idx + 1, frame->target_count);
        snprintf(lines[1], 19, "--------");
        snprintf(lines[2], 19, "X:  %5d mm", (int)t->x_mm);
        snprintf(lines[3], 19, "Y:  %5d mm", (int)t->y_mm);
        snprintf(lines[4], 19, "DST: %4u mm", (unsigned)t->distance_mm);
        snprintf(lines[5], 19, "SPD: %4d cm/s", (int)t->speed_cm_s);
        snprintf(lines[6], 19, "AZ:  %5.1f", (double)frame->azimuth_deg);
    } else {
        snprintf(lines[3], 19, "  NO TARGET");
    }

    /* Pad all lines to max_chars and null-terminate */
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < max_chars; ++j)
            if (lines[i][j] == '\0') lines[i][j] = ' ';
        lines[i][max_chars] = '\0';
    }

    /* Check against previous; only redraw changed lines */
    static char s_prev_lines[7][20];
    for (int i = 0; i < 7; ++i) {
        if (memcmp(lines[i], s_prev_lines[i], max_chars + 1) == 0) continue;
        memcpy(s_prev_lines[i], lines[i], max_chars + 1);
        draw_string(s_right, x0, y0 + i * line_h, lines[i],
                    COLOR_GREEN_BRIGHT, COLOR_BLACK, scale);
    }
}

/* ── Mode interface ──────────────────────────────────────────────── */

static void radar_enter(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right)
{
    s_left = left;
    s_right = right;
    s_sweep_start_us = esp_timer_get_time();
    s_last_sweep_deg = 0.0f;
    s_last_render_frame = 0;
    memset(s_targets, 0, sizeof(s_targets));

    /* Left eye normally uses 90° (swap=true, mx=true, my=false) for the
       eye-tracking mode. Add a vertical mirror so the FOV wedge/sweep
       point toward the bottom of the physical display. */
    esp_lcd_panel_swap_xy(s_left, true);
    esp_lcd_panel_mirror(s_left, true, true);

    /* Right eye normally uses 270° for paired-eye mirroring.
       For independent text content, match standard orientation. */
    esp_lcd_panel_swap_xy(s_right, true);
    esp_lcd_panel_mirror(s_right, true, true);

    build_luts();
    display_fill(s_left, COLOR_BLACK);
    draw_static_grid();
    display_fill(s_right, COLOR_BLACK);

    ESP_LOGI(TAG, "entered radar mode");
}

static void radar_render(esp_lcd_panel_handle_t left, esp_lcd_panel_handle_t right,
                         const mode_frame_t *frame)
{
    (void)left; (void)right;
    if (frame->frame_id == s_last_render_frame) return;
    s_last_render_frame = frame->frame_id;

    update_targets(frame, sweep_angle_now());
    render_left();
    render_right(frame);
}

static void radar_leave(void)
{
    /* Restore left eye to plain 90° for eye-tracking mode */
    esp_lcd_panel_swap_xy(s_left, true);
    esp_lcd_panel_mirror(s_left, true, false);

    /* Restore right eye to 270° for paired-eye mode */
    esp_lcd_panel_swap_xy(s_right, true);
    esp_lcd_panel_mirror(s_right, false, true);
}

const display_mode_t mode_radar = {
    .name   = "radar",
    .enter  = radar_enter,
    .render = radar_render,
    .leave  = radar_leave,
};
