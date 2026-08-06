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
/*
 * EyeN – deployment configuration
 *
 * This is the single file to edit when changing hardware layout, sensor
 * placement, or tuning parameters.  Rebuild after changes:  idf.py build
 */

#include "driver/spi_master.h"
#include "driver/uart.h"
#include "hal/adc_types.h"

namespace cfg
{

/* ── Display (dual GC9A01 on shared SPI bus) ─────────────────────── */

namespace lcd
{
inline constexpr int sclk = 18;
inline constexpr int mosi = 23;
inline constexpr int dc = 4;
inline constexpr int rst = 27;
inline constexpr int cs_left = 5;
inline constexpr int cs_right = 15;
inline constexpr auto spi_host = SPI2_HOST;
inline constexpr int h_res = 240;
inline constexpr int v_res = 240;
inline constexpr int spi_hz = 40'000'000;

/*
 * Panel rotation in degrees clockwise (0 / 90 / 180 / 270).
 * Round GC9A01 modules often have the flex on the outer edge of each
 * eye — use opposite 90°/270° so software "up" is world up on both.
 * If gaze still feels wrong, try swapping the two values, or 0/180.
 */
inline constexpr int left_rotation_deg = 90;
inline constexpr int right_rotation_deg = 270;
} // namespace lcd

/* ── Pupil / eye appearance ──────────────────────────────────────── */

namespace dot
{
inline constexpr int radius_min = 30; /* pupil at ≥ far range (px)         */
inline constexpr int radius_max = 55; /* pupil when close (px)             */
inline constexpr int near_mm = 400;   /* distance → max radius             */
inline constexpr int far_mm = 3000;   /* ≥3 m → min radius (no further shrink) */
} // namespace dot

namespace gaze
{
inline constexpr float max_deg = 50.0f;        /* ±degrees mapped to screen edge    */
inline constexpr float az_deadband_deg = 1.5f; /* ignore azimuth jitter below this  */
inline constexpr float coast_max_deg = 6.0f;   /* max coast away from last fix      */

/* Multi-person gaze (2–3 targets): dwell then rotate; motion steals focus */
inline constexpr int hold_min_ms = 2000;    /* random hold lower bound           */
inline constexpr int hold_max_ms = 5000;    /* random hold upper bound           */
inline constexpr int motion_cm_s = 15;      /* |speed| ≥ this → moving (steal)   */
inline constexpr int motion_lock_ms = 2000; /* min ms on a mover before another can steal */
} // namespace gaze

/* ── Radar (LD2450) common ───────────────────────────────────────── */

namespace radar
{
inline constexpr int baud = 256000;
} // namespace radar

/* ── Animation / smoothing ───────────────────────────────────────── */

namespace smooth
{
inline constexpr float h = 0.60f; /* horizontal lerp factor (0‥1)      */
inline constexpr float v = 0.40f; /* vertical lerp factor  (0‥1)      */
inline constexpr float r = 0.30f; /* pupil radius lerp factor (0‥1)    */
                                  /*   smaller = smoother / laggier    */
                                  /*   1.0 = no smoothing (instant)    */
} // namespace smooth

/* ── Radar frame / filter tuning ─────────────────────────────────── */

namespace frame
{
/* sensor outputs 10 Hz, so ~100ms between frames;
   long timeout to avoid missing queued frames in buffer */
inline constexpr int timeout_ms = 100;
inline constexpr int stale_ms = 500; /* drop a sensor's data after this   */
} // namespace frame

/* ── Software-side target filters ────────────────────────────────── */

namespace filter
{
inline constexpr int min_speed_cm_s = 0; /* |speed| below this → ghost (0=off)*/
inline constexpr int min_dist_mm = 400;  /* closer than this → phantom         */
inline constexpr int max_dist_mm = 6000; /* farther than this → noise          */
inline constexpr int persist_frames = 2; /* must appear N of last 3 frames     */
} // namespace filter

/* ── Mode button ─────────────────────────────────────────────────── */

namespace button
{
inline constexpr int gpio = 21; /* pull-up; button shorts to GND     */
inline constexpr int debounce_ms = 200;
} // namespace button

/* ── Radar display mode ──────────────────────────────────────────── */

namespace sweep
{
inline constexpr int ms = 3000; /* one full sweep revolution (ms)    */
} // namespace sweep

/* ── Single-sensor vertical: distance + potentiometer ────────────── *
 *
 * elevation = atan2(person_aim_mm - mount_height_mm, range_y_mm)
 * Potentiometer maps ADC → mount_height_mm so eye-level vs floor
 * placement is a dial, not a rebuild.
 */

namespace pot
{
inline constexpr int gpio = 34; /* ADC1 input-only                   */
inline constexpr auto adc_unit = ADC_UNIT_1;
inline constexpr auto adc_channel = ADC_CHANNEL_6; /* GPIO34 on ESP32 */
inline constexpr auto atten = ADC_ATTEN_DB_12;
inline constexpr int mount_min_mm = 0;     /* pot at 0%  → sensor on floor      */
inline constexpr int mount_max_mm = 2000;  /* pot at 100% → sensor ~2 m up       */
inline constexpr int person_aim_mm = 1500; /* assumed torso/face height (mm)     */
inline constexpr float smooth = 0.15f;     /* ADC lerp factor (0‥1)             */

} // namespace pot

/* ── Sensor configuration ────────────────────────────────────────── *
 *
 * Single LD2450 module on a dedicated UART.
 *
 *   name      – label for log messages
 *   uart_num  – ESP32 UART peripheral (UART_NUM_1 or UART_NUM_2)
 *   rx_gpio   – ESP32 GPIO receiving the sensor's TX line
 *   tx_gpio   – ESP32 GPIO to sensor RX, or -1 if not connected
 *   inverted  – true if the module is mounted upside-down;
 *               this negates the X axis so left/right stay correct
 */

struct SensorConfig
{
    const char *name;
    uart_port_t uart_num;
    int rx_gpio;
    int tx_gpio;
    bool inverted;
};

/* Default: one flat sensor on UART2 (RX2/TX2).
 * inverted=true negates radar X so left/right matches the eyes after
 * panel rotation (set false if gaze tracks the wrong way horizontally). */
inline constexpr SensorConfig sensor{"main", UART_NUM_2, 16, 17, true};

} // namespace cfg
