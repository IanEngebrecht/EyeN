#pragma once
/*
 * EyeN – deployment configuration
 *
 * This is the single file to edit when changing hardware layout, sensor
 * placement, or tuning parameters.  Rebuild after changes:  idf.py build
 *
 * Radar backend is chosen in main/CMakeLists.txt (RADAR_SRC):
 *   radar_stack_single.c  – default: one LD2450 + pot vertical
 *   radar_stack_multi.c   – pitch-stacked N-sensor fusion (archived)
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

/*
 * Panel rotation in degrees clockwise (0 / 90 / 180 / 270).
 * Round GC9A01 modules often have the flex on the outer edge of each
 * eye — use opposite 90°/270° so software “up” is world up on both.
 * If gaze still feels wrong, try swapping the two values, or 0/180.
 */
#define CFG_LCD_LEFT_ROTATION_DEG    90
#define CFG_LCD_RIGHT_ROTATION_DEG  270

/* ── Pupil / eye appearance ──────────────────────────────────────── */

#define CFG_DOT_RADIUS_MIN  30          /* pupil at ≥ far range (px)         */
#define CFG_DOT_RADIUS_MAX  55          /* pupil when close (px)             */
#define CFG_DOT_NEAR_MM     400         /* distance → max radius             */
#define CFG_DOT_FAR_MM      3000        /* ≥3 m → min radius (no further shrink) */
#define CFG_GAZE_MAX_DEG    50.0f       /* ±degrees mapped to screen edge    */
#define CFG_GAZE_AZ_DEADBAND_DEG  1.5f  /* ignore azimuth jitter below this  */
#define CFG_GAZE_COAST_MAX_DEG    6.0f  /* max coast away from last fix      */

/* Multi-person gaze (2–3 targets): dwell then rotate; motion steals focus */
#define CFG_GAZE_HOLD_MIN_MS     2000   /* random hold lower bound           */
#define CFG_GAZE_HOLD_MAX_MS     5000   /* random hold upper bound           */
#define CFG_GAZE_MOTION_CM_S       15   /* |speed| ≥ this → moving (steal)   */
#define CFG_GAZE_MOTION_LOCK_MS  2000   /* min ms on a mover before another can steal */

/* ── Radar (LD2450) common ───────────────────────────────────────── */

#define CFG_RADAR_BAUD      256000

/* ── Animation / smoothing ───────────────────────────────────────── */

#define CFG_SMOOTH_H        0.60f       /* horizontal lerp factor (0‥1)      */
#define CFG_SMOOTH_V        0.40f       /* vertical lerp factor  (0‥1)      */
#define CFG_SMOOTH_R        0.30f       /* pupil radius lerp factor (0‥1)    */
                                        /*   smaller = smoother / laggier    */
                                        /*   1.0 = no smoothing (instant)    */

/* ── Radar frame / filter tuning ─────────────────────────────────── */

#define CFG_FRAME_TIMEOUT_MS    15      /* short so animation loop stays fluid */
#define CFG_STALE_MS            500     /* drop a sensor's data after this   */

/* Multi-backend association (used only by radar_stack_multi.c) */
#define CFG_ASSOC_MAX_DX_MM     400
#define CFG_ASSOC_MAX_DDIST_MM  500

/* ── Software-side target filters ────────────────────────────────── */

#define CFG_FILTER_MIN_SPEED_CM_S   0   /* |speed| below this → ghost (0=off)*/
#define CFG_FILTER_MIN_DIST_MM    100   /* closer than this → phantom         */
#define CFG_FILTER_MAX_DIST_MM   4000   /* farther than this → noise          */
#define CFG_FILTER_PERSIST_FRAMES   2   /* must appear N of last 3 frames     */

/* ── Mux pins (multi backend; reserved for 74HC4051) ─────────────── */

#define CFG_MUX_S0_GPIO    -1
#define CFG_MUX_S1_GPIO    -1

/* ── Single-sensor vertical: distance + potentiometer ────────────── *
 *
 * elevation = atan2(person_aim_mm - mount_height_mm, range_y_mm)
 * Potentiometer maps ADC → mount_height_mm so eye-level vs floor
 * placement is a dial, not a rebuild.
 */

#define CFG_POT_GPIO            34      /* ADC1 input-only                   */
#define CFG_POT_ADC_UNIT        ADC_UNIT_1
#define CFG_POT_ADC_CHANNEL     ADC_CHANNEL_6  /* GPIO34 on ESP32            */
#define CFG_POT_ATTEN           ADC_ATTEN_DB_12
#define CFG_POT_MOUNT_MIN_MM    0       /* pot at 0%  → sensor on floor      */
#define CFG_POT_MOUNT_MAX_MM    2000    /* pot at 100% → sensor ~2 m up       */
#define CFG_PERSON_AIM_MM       1500    /* assumed torso/face height (mm)     */
#define CFG_POT_SMOOTH          0.15f   /* ADC lerp factor (0‥1)             */

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
 * Single backend uses the first enabled row only.
 * Multi backend sorts by pitch; bands = 2 × enabled_count − 1.
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
} eyen_sensor_cfg_t;

/* Default: one flat sensor on UART2 (RX2/TX2).
 * inverted=true negates radar X so left/right matches the eyes after
 * panel rotation (set false if gaze tracks the wrong way horizontally). */
#define CFG_SENSORS                                                           \
    { "main", UART_NUM_2, 16, 17, 0, true, -1, true },

/*
 * To restore pitch-stack multi-sensor (also set RADAR_SRC=radar_stack_multi.c):
 *
 * #define CFG_SENSORS                                                        \
 *     { "lower", UART_NUM_2, 16, 17, -20, false, -1, true  },               \
 *     { "upper", UART_NUM_1, 21, 22,  20, false, -1, true  },
 */
