#include "display.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "config.h"

static const char *TAG = "display";

#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000

static esp_lcd_panel_handle_t s_panel_l;
static esp_lcd_panel_handle_t s_panel_r;
static int  s_dot_x = -1;
static int  s_dot_y = -1;
static int  s_dot_r = 0;
static bool s_dot_valid;

static esp_err_t fill_panel(esp_lcd_panel_handle_t panel, uint16_t color)
{
    const size_t line_bytes = CFG_LCD_H_RES * sizeof(uint16_t);
    uint16_t *line = heap_caps_malloc(line_bytes, MALLOC_CAP_DMA);
    if (!line) return ESP_ERR_NO_MEM;
    for (int x = 0; x < CFG_LCD_H_RES; ++x) line[x] = color;
    esp_err_t err = ESP_OK;
    for (int y = 0; y < CFG_LCD_V_RES; ++y) {
        err = esp_lcd_panel_draw_bitmap(panel, 0, y, CFG_LCD_H_RES, y + 1, line);
        if (err != ESP_OK) break;
    }
    free(line);
    return err;
}

static void draw_filled_circle(esp_lcd_panel_handle_t panel, int cx, int cy,
                               int radius, uint16_t color)
{
    const int r2 = radius * radius;
    const int x0 = cx - radius;
    const int x1 = cx + radius;
    const int y0 = cy - radius;
    const int y1 = cy + radius;
    const int w  = x1 - x0 + 1;

    uint16_t *row = heap_caps_malloc((size_t)w * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!row) {
        ESP_LOGE(TAG, "no mem for circle row");
        return;
    }

    for (int y = y0; y <= y1; ++y) {
        if (y < 0 || y >= CFG_LCD_V_RES) continue;
        int draw_x0 = -1, draw_x1 = -1;
        for (int x = x0; x <= x1; ++x) {
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r2 && x >= 0 && x < CFG_LCD_H_RES) {
                if (draw_x0 < 0) draw_x0 = x;
                draw_x1 = x;
            }
        }
        if (draw_x0 < 0) continue;
        const int n = draw_x1 - draw_x0 + 1;
        for (int i = 0; i < n; ++i) row[i] = color;
        esp_lcd_panel_draw_bitmap(panel, draw_x0, y, draw_x1 + 1, y + 1, row);
    }
    free(row);
}

static void draw_dot_on_both(int x, int y, int radius, uint16_t color)
{
    draw_filled_circle(s_panel_l, x, y, radius, color);
    draw_filled_circle(s_panel_r, x, y, radius, color);
}

/*
 * Flicker-free dot move/resize: render the bounding box that covers both old
 * and new circles in a single pass.  Each row is sent with its final pixels
 * (white background + black new circle) so there is no visible erase gap.
 */
static void move_dot_on_panel(esp_lcd_panel_handle_t panel,
                              int ox, int oy, int orad,
                              int nx, int ny, int nrad)
{
    const int nr2 = nrad * nrad;
    const int margin = 2;

    int bx0 = (ox - orad < nx - nrad ? ox - orad : nx - nrad) - margin;
    int bx1 = (ox + orad > nx + nrad ? ox + orad : nx + nrad) + margin;
    int by0 = (oy - orad < ny - nrad ? oy - orad : ny - nrad) - margin;
    int by1 = (oy + orad > ny + nrad ? oy + orad : ny + nrad) + margin;

    if (bx0 < 0)              bx0 = 0;
    if (bx1 >= CFG_LCD_H_RES) bx1 = CFG_LCD_H_RES - 1;
    if (by0 < 0)              by0 = 0;
    if (by1 >= CFG_LCD_V_RES) by1 = CFG_LCD_V_RES - 1;

    const int w = bx1 - bx0 + 1;
    uint16_t *row = heap_caps_malloc((size_t)w * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!row) {
        ESP_LOGE(TAG, "no mem for move row");
        return;
    }

    for (int y = by0; y <= by1; ++y) {
        for (int i = 0; i < w; ++i) {
            int x = bx0 + i;
            int dx = x - nx, dy = y - ny;
            row[i] = (dx * dx + dy * dy <= nr2) ? COLOR_BLACK : COLOR_WHITE;
        }
        esp_lcd_panel_draw_bitmap(panel, bx0, y, bx1 + 1, y + 1, row);
    }
    free(row);
}

/*
 * Map clockwise rotation → MADCTL via swap_xy + mirror.
 *   0°:   MV=0 MX=0 MY=0
 *  90°:   MV=1 MX=1 MY=0
 * 180°:   MV=0 MX=1 MY=1
 * 270°:   MV=1 MX=0 MY=1
 */
static esp_err_t apply_rotation(esp_lcd_panel_handle_t panel, int deg)
{
    bool swap = false, mx = false, my = false;
    switch (((deg % 360) + 360) % 360) {
    case 0:   break;
    case 90:  swap = true;  mx = true;  break;
    case 180:               mx = true;  my = true; break;
    case 270: swap = true;              my = true; break;
    default:
        ESP_LOGW(TAG, "unsupported rotation %d° (use 0/90/180/270); using 0", deg);
        break;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, swap), TAG, "swap_xy failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, mx, my), TAG, "mirror failed");
    return ESP_OK;
}

static esp_err_t create_panel(spi_host_device_t host, int cs_gpio,
                              int rotation_deg, esp_lcd_panel_handle_t *out_panel)
{
    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num     = cs_gpio,
        .dc_gpio_num     = CFG_LCD_DC,
        .spi_mode        = 0,
        .pclk_hz         = CFG_LCD_SPI_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host,
                                                 &io_config, &io),
                        TAG, "panel io failed");

    /* rst_gpio_num = -1: shared RST is pulsed once in display_init().
     * Giving RST only to the left panel used to HW-reset both mid-sequence,
     * then SW-reset the right — flaky after soft reboot / re-flash. */
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num    = -1,
        .rgb_ele_order     = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel    = 16,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_gc9a01(io, &panel_config, &panel),
                        TAG, "gc9a01 create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel),          TAG, "reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel),           TAG, "init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "invert failed");
    ESP_RETURN_ON_ERROR(apply_rotation(panel, rotation_deg), TAG, "rotation failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true),  TAG, "disp on failed");

    *out_panel = panel;
    return ESP_OK;
}

/** Hardware-reset both GC9A01s via the shared RST line (active-low). */
static esp_err_t pulse_shared_reset(void)
{
    const gpio_config_t io_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << CFG_LCD_RST,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "RST gpio config failed");

    gpio_set_level(CFG_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CFG_LCD_RST, 1);
    /* Datasheet-ish settle after POR; soft reboot needs this more than cold power. */
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "Init SPI bus for dual GC9A01");
    const spi_bus_config_t bus_config = {
        .sclk_io_num   = CFG_LCD_SCLK,
        .mosi_io_num   = CFG_LCD_MOSI,
        .miso_io_num   = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CFG_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CFG_LCD_SPI_HOST, &bus_config,
                                           SPI_DMA_CH_AUTO),
                        TAG, "spi bus init failed");

    ESP_RETURN_ON_ERROR(pulse_shared_reset(), TAG, "shared RST failed");

    ESP_RETURN_ON_ERROR(create_panel(CFG_LCD_SPI_HOST, CFG_LCD_CS_LEFT,
                                     CFG_LCD_LEFT_ROTATION_DEG, &s_panel_l),
                        TAG, "left panel failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(create_panel(CFG_LCD_SPI_HOST, CFG_LCD_CS_RIGHT,
                                     CFG_LCD_RIGHT_ROTATION_DEG, &s_panel_r),
                        TAG, "right panel failed");

    /* Re-assert display-on after both inits (helps after soft reboot). */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_l, true), TAG, "left on");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_r, true), TAG, "right on");

    display_clear();
    ESP_LOGI(TAG, "Both eyes ready (%dx%d) rot L=%d° R=%d°",
             CFG_LCD_H_RES, CFG_LCD_V_RES,
             CFG_LCD_LEFT_ROTATION_DEG, CFG_LCD_RIGHT_ROTATION_DEG);
    return ESP_OK;
}

void display_clear(void)
{
    fill_panel(s_panel_l, COLOR_WHITE);
    fill_panel(s_panel_r, COLOR_WHITE);
    s_dot_valid = false;
    s_dot_x = -1;
    s_dot_y = -1;
    s_dot_r = 0;
}

void display_set_dot(int x, int y, int radius)
{
    if (radius < 1) radius = 1;
    if (radius > CFG_DOT_RADIUS_MAX) radius = CFG_DOT_RADIUS_MAX;

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

    if (s_dot_valid) {
        move_dot_on_panel(s_panel_l, s_dot_x, s_dot_y, s_dot_r, x, y, radius);
        move_dot_on_panel(s_panel_r, s_dot_x, s_dot_y, s_dot_r, x, y, radius);
    } else {
        draw_dot_on_both(x, y, radius, COLOR_BLACK);
    }
    s_dot_x = x;
    s_dot_y = y;
    s_dot_r = radius;
    s_dot_valid = true;
}
