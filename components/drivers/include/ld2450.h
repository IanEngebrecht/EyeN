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

#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "driver/uart.h"
#include "esp_err.h"

struct Target
{
    int16_t x_mm{};
    int16_t y_mm{};
    int16_t speed_cm_s{};
    uint16_t distance_mm{};
    bool valid{};
};

struct Zone
{
    int16_t x1{}, y1{}, x2{}, y2{};
};

class Ld2450
{
  public:
    struct Config
    {
        uart_port_t uart_num{};
        int tx_gpio{-1};
        int rx_gpio{};
        int baud{};
    };

    static constexpr int target_count = 3;

    Ld2450() = default;
    ~Ld2450();

    Ld2450(Ld2450 &&other) noexcept;
    Ld2450 &operator=(Ld2450 &&other) noexcept;

    Ld2450(const Ld2450 &) = delete;
    Ld2450 &operator=(const Ld2450 &) = delete;

    static esp_err_t create(const Config &cfg, Ld2450 &out);

    bool valid() const { return valid_; }
    uart_port_t uart_num() const { return uart_num_; }

    esp_err_t read_frame(std::array<Target, target_count> &targets, int timeout_ms);

    void unstick();
    esp_err_t read_firmware_version(std::span<char> buf);
    esp_err_t set_sensitivity(uint8_t val);
    esp_err_t set_energy_threshold(uint16_t val);
    esp_err_t set_speed_filter(uint16_t val);
    esp_err_t set_hold_time(uint16_t val);
    esp_err_t set_zones(uint16_t type, const std::array<Zone, 3> &zones);
    esp_err_t restart();
    esp_err_t set_output_rate(uint8_t fps);

  private:
    uart_port_t uart_num_{};
    bool valid_{};

    void release();

    esp_err_t send_raw(const uint8_t *data, int len);
    esp_err_t wait_ack(uint16_t *out_status);
    void force_end_config();
    esp_err_t enter_config();
    esp_err_t end_config();
    esp_err_t send_config_cmd(const uint8_t *payload, int payload_len);
};
