#pragma once
/*
 * EyeAN – deployment configuration
 *
 * This is the single file to edit when changing hardware layout, sensor
 * placement, or tuning parameters.  Rebuild after changes:  idf.py build
 */

#include <stdbool.h>
#include "driver/uart.h"

/* ── Display (dual GC9A01 on shared SPI bus) ─────────────────────── */

#define CFG_LCD_SCLK        18
#define CFG_LCD_MOSI        23
#define CFG_LCD_DC          27
#define CFG_LCD_RST          4
#define CFG_LCD_CS_LEFT      5
#define CFG_LCD_CS_RIGHT    15
#define CFG_LCD_SPI_HOST    SPI2_HOST
#define CFG_LCD_H_RES      240
#define CFG_LCD_V_RES      240
#define CFG_LCD_SPI_HZ     (40 * 1000 * 1000)

/* ── Pupil / eye appearance ──────────────────────────────────────── */

#define CFG_DOT_RADIUS      30          /* pupil size in pixels              */
#define CFG_GAZE_MAX_DEG    50.0f       /* ±degrees mapped to screen edge    */

/* ── Radar (LD2450) common ───────────────────────────────────────── */

#define CFG_RADAR_BAUD      256000

/* ── Animation / smoothing ───────────────────────────────────────── */

#define CFG_SMOOTH_H        0.18f       /* horizontal lerp factor (0‥1)      */
#define CFG_SMOOTH_V        0.12f       /* vertical lerp factor  (0‥1)      */
                                        /*   smaller = smoother / laggier    */
                                        /*   1.0 = no smoothing (instant)    */

/* ── Radar-stack fusion tuning ───────────────────────────────────── */

#define CFG_FRAME_TIMEOUT_MS    15      /* short so animation loop stays fluid */
#define CFG_STALE_MS            500     /* drop a sensor's data after this   */
#define CFG_ASSOC_MAX_DX_MM     400     /* max X delta to associate targets  */
#define CFG_ASSOC_MAX_DDIST_MM  500     /* max distance delta for association*/

/* ── Software-side target filters (applied before fusion) ────────── */

#define CFG_FILTER_MIN_SPEED_CM_S   0   /* |speed| below this → ghost (0=off)*/
#define CFG_FILTER_MIN_DIST_MM    100   /* closer than this → phantom         */
#define CFG_FILTER_MAX_DIST_MM   4000   /* farther than this → noise          */
#define CFG_FILTER_PERSIST_FRAMES   2   /* must appear N of last 3 frames     */

/* ── Mux pins (reserved; set to real GPIOs when 74HC4051 is wired) ─ */

#define CFG_MUX_S0_GPIO    -1
#define CFG_MUX_S1_GPIO    -1

/* ── Sensor array ────────────────────────────────────────────────── *
 *
 * Each row defines one LD2450 module.  Fields:
 *
 *   name        – label for log messages
 *   uart_num    – ESP32 UART peripheral (UART_NUM_1 or UART_NUM_2)
 *   rx_gpio     – ESP32 GPIO receiving the sensor's TX line
 *   tx_gpio     – ESP32 GPIO to sensor RX, or -1 if not connected
 *   pitch_deg   – physical tilt relative to horizon (positive = up)
 *   inverted    – true if the module is mounted upside-down;
 *                 this negates the X axis so left/right stay correct
 *   mux_channel – analog-mux channel (>=0), or -1 for dedicated UART
 *   enabled     – false to skip this sensor without removing the row
 *
 * Order does not matter — the radar stack sorts by pitch at startup.
 * Vertical bands are computed automatically:  2 × enabled_count − 1.
 */

typedef struct {
    const char *name;
    uart_port_t uart_num;
    int         rx_gpio;
    int         tx_gpio;
    int         pitch_deg;
    bool        inverted;
    int         mux_channel;
    bool        enabled;
} eyean_sensor_cfg_t;

#define CFG_SENSORS                                                           \
    { "lower", UART_NUM_2, 16, 17, -20, false, -1, true  },                  \
    { "upper", UART_NUM_1, 21, 22,  20, false, -1, true  },
