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

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int16_t x_mm;         /* +right / -left when facing the antenna */
        int16_t y_mm;         /* +away from sensor (forward) */
        int16_t speed_cm_s;   /* +away / -toward */
        uint16_t distance_mm; /* computed from x,y */
        bool valid;
    } ld2450_target_t;

    typedef struct
    {
        uart_port_t uart_num;
        int tx_gpio; /* -1 if ESP never TX to module */
        int rx_gpio;
        int baud;
    } ld2450_config_t;

    typedef struct ld2450_dev ld2450_dev_t;

    esp_err_t ld2450_create(const ld2450_config_t *cfg, ld2450_dev_t **out);
    void ld2450_destroy(ld2450_dev_t *dev);

    /**
     * Blocking read of the next radar frame on this device.
     * @param timeout_ms  Max wait while hunting for a frame header.
     */
    esp_err_t ld2450_read_frame(ld2450_dev_t *dev, ld2450_target_t targets[3], int timeout_ms);

    /* ── Configuration commands (require TX pin wired to sensor RX) ─── */

    /** Force end_config in case a sensor is stuck in config mode. Safe to call anytime. */
    void ld2450_unstick(ld2450_dev_t *dev);

    /** Read firmware version string (e.g. "V2.02.23090616"). */
    esp_err_t ld2450_read_firmware_version(ld2450_dev_t *dev, char *buf, int buf_len);

    /** Set sensitivity across all 8 range gates. val: 0–9. */
    esp_err_t ld2450_set_sensitivity(ld2450_dev_t *dev, uint8_t val);

    /** Set energy threshold. val: 100–10000. Higher = fewer ghost detections. */
    esp_err_t ld2450_set_energy_threshold(ld2450_dev_t *dev, uint16_t val);

    /** Set minimum speed filter. val: 0–100 cm/s. Targets slower than this are ignored. */
    esp_err_t ld2450_set_speed_filter(ld2450_dev_t *dev, uint16_t val);

    /** Set target hold time. val: 0–60 seconds. How long target persists after leaving FOV. */
    esp_err_t ld2450_set_hold_time(ld2450_dev_t *dev, uint16_t val);

    /**
     * Set zone filtering. type: 0=off, 1=detect-only, 2=exclude.
     * Each zone is a rectangle (x1,y1)–(x2,y2) in mm.  Up to 3 zones.
     * Pass x1==x2==y1==y2==0 for unused zones.
     */
    typedef struct
    {
        int16_t x1, y1, x2, y2;
    } ld2450_zone_t;

    esp_err_t ld2450_set_zones(ld2450_dev_t *dev, uint16_t type, const ld2450_zone_t zones[3]);

    /** Restart the module (apply settings that need a reboot). */
    esp_err_t ld2450_restart(ld2450_dev_t *dev);

    /** Set the output frame rate in FPS (typically 1-25). */
    esp_err_t ld2450_set_output_rate(ld2450_dev_t *dev, uint8_t fps);

#ifdef __cplusplus
}
#endif
