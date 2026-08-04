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

static esp_lcd_panel_handle_t s_panel_l;
static esp_lcd_panel_handle_t s_panel_r;

void display_fill(esp_lcd_panel_handle_t panel, uint16_t color)
{
    const size_t line_bytes = CFG_LCD_H_RES * sizeof(uint16_t);
    uint16_t *line = heap_caps_malloc(line_bytes, MALLOC_CAP_DMA);
    if (!line) return;
    for (int x = 0; x < CFG_LCD_H_RES; ++x) line[x] = color;
    for (int y = 0; y < CFG_LCD_V_RES; ++y) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, CFG_LCD_H_RES, y + 1, line);
    }
    free(line);
}

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

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_l, true), TAG, "left on");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_r, true), TAG, "right on");

    display_fill(s_panel_l, 0xFFFF);
    display_fill(s_panel_r, 0xFFFF);

    ESP_LOGI(TAG, "Both eyes ready (%dx%d) rot L=%d° R=%d°",
             CFG_LCD_H_RES, CFG_LCD_V_RES,
             CFG_LCD_LEFT_ROTATION_DEG, CFG_LCD_RIGHT_ROTATION_DEG);
    return ESP_OK;
}

esp_lcd_panel_handle_t display_get_panel_left(void)  { return s_panel_l; }
esp_lcd_panel_handle_t display_get_panel_right(void) { return s_panel_r; }
