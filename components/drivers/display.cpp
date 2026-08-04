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

#include <algorithm>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

static const char *TAG = "display";

void Display::fill(esp_lcd_panel_handle_t panel, uint16_t color)
{
    std::fill(std::begin(dma_line_), std::end(dma_line_), color);
    for (int y = 0; y < cfg::lcd::v_res; ++y)
        esp_lcd_panel_draw_bitmap(panel, 0, y, cfg::lcd::h_res, y + 1, dma_line_);
}

esp_err_t Display::apply_rotation(esp_lcd_panel_handle_t panel, int deg)
{
    bool swap = false, mx = false, my = false;
    switch (((deg % 360) + 360) % 360)
    {
    case 0:
        break;
    case 90:
        swap = true;
        mx = true;
        break;
    case 180:
        mx = true;
        my = true;
        break;
    case 270:
        swap = true;
        my = true;
        break;
    default:
        ESP_LOGW(TAG, "unsupported rotation %d° (use 0/90/180/270); using 0", deg);
        break;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, swap), TAG, "swap_xy failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, mx, my), TAG, "mirror failed");
    return ESP_OK;
}

esp_err_t Display::create_panel(spi_host_device_t host, int cs_gpio, int rotation_deg,
                                esp_lcd_panel_handle_t *out_panel)
{
    esp_lcd_panel_io_handle_t io = nullptr;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = cs_gpio,
        .dc_gpio_num = cfg::lcd::dc,
        .spi_mode = 0,
        .pclk_hz = cfg::lcd::spi_hz,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(host), &io_config, &io), TAG,
        "panel io failed");

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_handle_t panel = nullptr;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_gc9a01(io, &panel_config, &panel), TAG,
                        "gc9a01 create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "invert failed");
    ESP_RETURN_ON_ERROR(apply_rotation(panel, rotation_deg), TAG, "rotation failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "disp on failed");

    *out_panel = panel;
    return ESP_OK;
}

esp_err_t Display::pulse_shared_reset()
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << cfg::lcd::rst,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "RST gpio config failed");

    gpio_set_level(static_cast<gpio_num_t>(cfg::lcd::rst), 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(static_cast<gpio_num_t>(cfg::lcd::rst), 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

esp_err_t Display::init()
{
    ESP_LOGI(TAG, "Init SPI bus for dual GC9A01");
    const spi_bus_config_t bus_config = {
        .mosi_io_num = cfg::lcd::mosi,
        .miso_io_num = -1,
        .sclk_io_num = cfg::lcd::sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = cfg::lcd::h_res * 80 * static_cast<int>(sizeof(uint16_t)),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg::lcd::spi_host, &bus_config, SPI_DMA_CH_AUTO), TAG,
                        "spi bus init failed");

    ESP_RETURN_ON_ERROR(pulse_shared_reset(), TAG, "shared RST failed");

    ESP_RETURN_ON_ERROR(
        create_panel(cfg::lcd::spi_host, cfg::lcd::cs_left, cfg::lcd::left_rotation_deg, &left_),
        TAG, "left panel failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(
        create_panel(cfg::lcd::spi_host, cfg::lcd::cs_right, cfg::lcd::right_rotation_deg, &right_),
        TAG, "right panel failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(left_, true), TAG, "left on");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(right_, true), TAG, "right on");

    fill(left_, 0xFFFF);
    fill(right_, 0xFFFF);

    ESP_LOGI(TAG, "Both eyes ready (%dx%d) rot L=%d° R=%d°", cfg::lcd::h_res, cfg::lcd::v_res,
             cfg::lcd::left_rotation_deg, cfg::lcd::right_rotation_deg);
    return ESP_OK;
}
