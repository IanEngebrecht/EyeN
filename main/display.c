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

static void draw_dot_on_both(int x, int y, uint16_t color)
{
    draw_filled_circle(s_panel_l, x, y, CFG_DOT_RADIUS, color);
    draw_filled_circle(s_panel_r, x, y, CFG_DOT_RADIUS, color);
}

/*
 * Flicker-free dot move: render the bounding box that covers both old and new
 * circles in a single pass.  Each row is sent with its final pixels (white
 * background + black new circle) so there is no visible erase gap.
 */
static void move_dot_on_panel(esp_lcd_panel_handle_t panel,
                              int ox, int oy, int nx, int ny, int radius)
{
    const int r2 = radius * radius;
    const int margin = 2;

    int bx0 = (ox < nx ? ox : nx) - radius - margin;
    int bx1 = (ox > nx ? ox : nx) + radius + margin;
    int by0 = (oy < ny ? oy : ny) - radius - margin;
    int by1 = (oy > ny ? oy : ny) + radius + margin;

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
            row[i] = (dx * dx + dy * dy <= r2) ? COLOR_BLACK : COLOR_WHITE;
        }
        esp_lcd_panel_draw_bitmap(panel, bx0, y, bx1 + 1, y + 1, row);
    }
    free(row);
}

static esp_err_t create_panel(spi_host_device_t host, int cs_gpio, int rst_gpio,
                              esp_lcd_panel_handle_t *out_panel)
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

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num    = rst_gpio,
        .rgb_ele_order     = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel    = 16,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_gc9a01(io, &panel_config, &panel),
                        TAG, "gc9a01 create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel),          TAG, "reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel),           TAG, "init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, false, false), TAG, "mirror failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true),  TAG, "disp on failed");

    *out_panel = panel;
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

    ESP_RETURN_ON_ERROR(create_panel(CFG_LCD_SPI_HOST, CFG_LCD_CS_LEFT,
                                     CFG_LCD_RST, &s_panel_l),
                        TAG, "left panel failed");
    ESP_RETURN_ON_ERROR(create_panel(CFG_LCD_SPI_HOST, CFG_LCD_CS_RIGHT,
                                     -1, &s_panel_r),
                        TAG, "right panel failed");

    display_clear();
    ESP_LOGI(TAG, "Both eyes ready (%dx%d)", CFG_LCD_H_RES, CFG_LCD_V_RES);
    return ESP_OK;
}

void display_clear(void)
{
    fill_panel(s_panel_l, COLOR_WHITE);
    fill_panel(s_panel_r, COLOR_WHITE);
    s_dot_valid = false;
    s_dot_x = -1;
    s_dot_y = -1;
}

void display_set_dot(int x, int y)
{
    if (x < CFG_DOT_RADIUS)
        x = CFG_DOT_RADIUS;
    else if (x >= CFG_LCD_H_RES - CFG_DOT_RADIUS)
        x = CFG_LCD_H_RES - CFG_DOT_RADIUS - 1;
    if (y < CFG_DOT_RADIUS)
        y = CFG_DOT_RADIUS;
    else if (y >= CFG_LCD_V_RES - CFG_DOT_RADIUS)
        y = CFG_LCD_V_RES - CFG_DOT_RADIUS - 1;

    if (s_dot_valid && s_dot_x == x && s_dot_y == y) return;

    if (s_dot_valid) {
        move_dot_on_panel(s_panel_l, s_dot_x, s_dot_y, x, y, CFG_DOT_RADIUS);
        move_dot_on_panel(s_panel_r, s_dot_x, s_dot_y, x, y, CFG_DOT_RADIUS);
    } else {
        draw_dot_on_both(x, y, COLOR_BLACK);
    }
    s_dot_x = x;
    s_dot_y = y;
    s_dot_valid = true;
}
