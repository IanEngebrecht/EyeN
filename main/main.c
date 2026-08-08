#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "display.h"
#include "ld2450.h"
#include "radar_stack.h"

static const char *TAG = "EyeN";

#define CMD_BUF_SIZE 128

/* Azimuth hold + coast between 10 Hz radar frames. */
static bool s_az_have;
static float s_az_fixed;      /* last accepted radar azimuth (deg) */
static float s_az_rate;       /* deg/s from recent accepted steps   */
static int64_t s_az_fixed_us; /* time of last real radar frame      */
static uint32_t s_az_last_frame;

static void clamp_to_circle(float *x, float *y)
{
    const float cx = (CFG_LCD_H_RES - 1) * 0.5f;
    const float cy = (CFG_LCD_V_RES - 1) * 0.5f;
    /* Use max radius so a large close-up pupil never clips the rim. */
    const float max_r = (CFG_LCD_H_RES * 0.5f) - (float)CFG_DOT_RADIUS_MAX - 2.0f;
    const float dx = *x - cx;
    const float dy = *y - cy;
    const float dist = sqrtf(dx * dx + dy * dy);
    if (dist > max_r && dist > 0.0f)
    {
        const float s = max_r / dist;
        *x = cx + dx * s;
        *y = cy + dy * s;
    }
}

/** Closer → larger pupil. Distance outside [NEAR, FAR] clamps to the ends. */
static float radius_from_distance_mm(uint16_t dist_mm)
{
    float d = (float)dist_mm;
    if (d <= (float)CFG_DOT_NEAR_MM)
        return (float)CFG_DOT_RADIUS_MAX;
    if (d >= (float)CFG_DOT_FAR_MM)
        return (float)CFG_DOT_RADIUS_MIN;
    float t = (d - (float)CFG_DOT_NEAR_MM) / (float)(CFG_DOT_FAR_MM - CFG_DOT_NEAR_MM);
    return (float)CFG_DOT_RADIUS_MAX + t * (float)(CFG_DOT_RADIUS_MIN - CFG_DOT_RADIUS_MAX);
}

static void gaze_xy_from_az_elev(float az_deg, float elev_norm, float *tx, float *ty)
{
    float deg = az_deg;
    if (deg < -CFG_GAZE_MAX_DEG)
        deg = -CFG_GAZE_MAX_DEG;
    else if (deg > CFG_GAZE_MAX_DEG)
        deg = CFG_GAZE_MAX_DEG;

    *tx = ((deg + CFG_GAZE_MAX_DEG) / (2.0f * CFG_GAZE_MAX_DEG)) * (float)(CFG_LCD_H_RES - 1);

    float elev = elev_norm;
    if (elev < 0.0f)
        elev = 0.0f;
    else if (elev > 1.0f)
        elev = 1.0f;
    *ty = (1.0f - elev) * (float)(CFG_LCD_V_RES - 1);
    clamp_to_circle(tx, ty);
}

/**
 * Deadband noisy azimuth steps; estimate rate on accepted steps; coast
 * between real radar frames (frame_id bumps) so walking doesn't wait on 10 Hz.
 */
static float aim_azimuth_deg(const radar_gaze_t *g)
{
    const int64_t now = esp_timer_get_time();
    const float az = g->azimuth_deg;

    if (!s_az_have)
    {
        s_az_fixed = az;
        s_az_rate = 0.0f;
        s_az_fixed_us = now;
        s_az_last_frame = g->frame_id;
        s_az_have = true;
        return az;
    }

    if (g->frame_id != s_az_last_frame)
    {
        const float daz = az - s_az_fixed;
        const float dt = (float)(now - s_az_fixed_us) * 1e-6f;

        if (fabsf(daz) >= CFG_GAZE_AZ_DEADBAND_DEG)
        {
            if (dt > 0.04f && dt < 0.6f)
            {
                const float inst = daz / dt;
                s_az_rate += (inst - s_az_rate) * 0.45f;
            }
            s_az_fixed = az;
        }
        else
        {
            /* Below deadband: treat as stationary → kill coast rate. */
            s_az_rate *= 0.35f;
        }
        s_az_fixed_us = now;
        s_az_last_frame = g->frame_id;
    }

    float age = (float)(now - s_az_fixed_us) * 1e-6f;
    if (age < 0.0f)
        age = 0.0f;
    if (age > 0.25f)
        age = 0.25f; /* don't coast across long gaps */

    float coast = s_az_rate * age;
    if (coast > CFG_GAZE_COAST_MAX_DEG)
        coast = CFG_GAZE_COAST_MAX_DEG;
    else if (coast < -CFG_GAZE_COAST_MAX_DEG)
        coast = -CFG_GAZE_COAST_MAX_DEG;

    return s_az_fixed + coast;
}

static void reset_azimuth_coast(void)
{
    s_az_have = false;
    s_az_rate = 0.0f;
}

/*
 * Emit a single machine-parseable line with all sensor data + eye position.
 *
 * Format:
 *   $FRAME t=<ms> eye=<x>,<y> <name>=<count>:<x,y,d,spd>|<x,y,d,spd>|... ...
 *
 * Invalid target slots are written as "-".
 */
static void log_frame(const radar_gaze_t *g, int eye_x, int eye_y)
{
    char buf[512];
    int pos = 0;
    const int cap = (int)sizeof(buf) - 1;

    uint32_t ms = esp_log_timestamp();
    pos +=
        snprintf(buf + pos, cap - pos, "$FRAME t=%lu eye=%d,%d", (unsigned long)ms, eye_x, eye_y);

    for (int s = 0; s < g->slot_count && pos < cap; ++s)
    {
        const radar_slot_info_t *sl = &g->slots[s];
        pos += snprintf(buf + pos, cap - pos, " %s=%d:", sl->name, sl->target_count);
        for (int t = 0; t < RADAR_TARGETS_PER_SLOT && pos < cap; ++t)
        {
            if (t > 0)
                buf[pos++] = '|';
            const ld2450_target_t *tg = &sl->targets[t];
            if (tg->valid)
            {
                pos += snprintf(buf + pos, cap - pos, "%d,%d,%u,%d", (int)tg->x_mm, (int)tg->y_mm,
                                (unsigned)tg->distance_mm, (int)tg->speed_cm_s);
            }
            else
            {
                buf[pos++] = '-';
            }
        }
    }
    buf[pos] = '\0';
    ESP_LOGI(TAG, "%s", buf);
}

/*
 * Process a "$SET <param> <value>" command received from the host over UART0.
 * Applies the setting to ALL enabled sensors.
 */
static void handle_cmd(const char *line)
{
    char param[32] = {0};
    int value = 0;

    if (sscanf(line, "$SET %31s %d", param, &value) != 2)
        return;

    /* Software-side filters (applied on ESP32, always work) */
    radar_filter_cfg_t filt;
    radar_stack_get_filter(&filt);
    bool is_sw = true;

    if (strcmp(param, "min_speed") == 0)
    {
        filt.min_speed_cm_s = value;
    }
    else if (strcmp(param, "min_dist") == 0)
    {
        filt.min_dist_mm = value;
    }
    else if (strcmp(param, "max_dist") == 0)
    {
        filt.max_dist_mm = value;
    }
    else if (strcmp(param, "persist") == 0)
    {
        filt.persist_frames = value;
    }
    else
    {
        is_sw = false;
    }

    if (is_sw)
    {
        radar_stack_set_filter(&filt);
        ESP_LOGI(TAG, "$ACK %s %d OK", param, value);
        return;
    }

    /* Hardware sensor commands (require TX pin + firmware support) */
    int n = radar_stack_slot_count();
    esp_err_t err = ESP_OK;

    for (int i = 0; i < n; i++)
    {
        ld2450_dev_t *dev = radar_stack_get_dev(i);
        if (!dev)
            continue;

        esp_err_t e = ESP_ERR_NOT_SUPPORTED;
        if (strcmp(param, "sensitivity") == 0)
        {
            e = ld2450_set_sensitivity(dev, (uint8_t)value);
        }
        else if (strcmp(param, "energy") == 0)
        {
            e = ld2450_set_energy_threshold(dev, (uint16_t)value);
        }
        else if (strcmp(param, "speed_filter") == 0)
        {
            e = ld2450_set_speed_filter(dev, (uint16_t)value);
        }
        else if (strcmp(param, "hold_time") == 0)
        {
            e = ld2450_set_hold_time(dev, (uint16_t)value);
        }
        else if (strcmp(param, "restart") == 0)
        {
            e = ld2450_restart(dev);
        }
        if (e != ESP_OK)
            err = e;
    }

    if (err == ESP_OK)
        ESP_LOGI(TAG, "$ACK %s %d OK", param, value);
    else if (err == ESP_ERR_NOT_SUPPORTED)
        ESP_LOGW(TAG, "$ACK %s UNKNOWN_PARAM", param);
    else
        ESP_LOGW(TAG, "$ACK %s %d FAIL", param, value);
}

static char s_cmd_buf[CMD_BUF_SIZE];
static int s_cmd_pos = 0;

/* ---- Performance instrumentation ---- */
#define PERF_INTERVAL_US (5 * 1000 * 1000) /* 5 seconds */
static int64_t s_perf_start_us;
static uint32_t s_perf_frame_count;
static int64_t s_perf_frame_min_us;
static int64_t s_perf_frame_max_us;
static int64_t s_perf_frame_sum_us;
static int64_t s_perf_frame_begin_us;

static void poll_uart0_commands(void)
{
    uint8_t b;
    while (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1)
    {
        if (b == '\n' || b == '\r')
        {
            if (s_cmd_pos > 0)
            {
                s_cmd_buf[s_cmd_pos] = '\0';
                handle_cmd(s_cmd_buf);
                s_cmd_pos = 0;
            }
        }
        else if (s_cmd_pos < CMD_BUF_SIZE - 1)
        {
            s_cmd_buf[s_cmd_pos++] = (char)b;
        }
    }
}

static void init_uart0_rx(void)
{
    /* Install a driver on UART0 so we can read incoming bytes.
     * ESP-IDF logging/printf still works through the driver. */
    const int buf = 512;
    esp_err_t err = uart_driver_install(UART_NUM_0, buf, 0, 0, NULL, 0);
    if (err == ESP_ERR_INVALID_STATE)
    {
        /* Driver already installed (some IDF versions do this). */
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "uart0 driver install: %s", esp_err_to_name(err));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "EyeN starting");

    init_uart0_rx();
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(radar_stack_init());

    const float center_x = (float)(CFG_LCD_H_RES / 2);
    const float center_y = (float)(CFG_LCD_V_RES / 2);

    float cur_x = center_x, cur_y = center_y;
    float tgt_x = center_x, tgt_y = center_y;
    float cur_r = (float)CFG_DOT_RADIUS_MIN;
    float tgt_r = cur_r;

    display_set_dot((int)lroundf(cur_x), (int)lroundf(cur_y), (int)lroundf(cur_r));

    bool last_human = false;
    int log_skip = 0;

    /* Initialize perf counters */
    s_perf_start_us = esp_timer_get_time();
    s_perf_frame_count = 0;
    s_perf_frame_min_us = INT64_MAX;
    s_perf_frame_max_us = 0;
    s_perf_frame_sum_us = 0;
    s_perf_frame_begin_us = s_perf_start_us;

    while (true)
    {
        s_perf_frame_begin_us = esp_timer_get_time();

        poll_uart0_commands();

        radar_gaze_t gaze;
        esp_err_t err = radar_stack_update(&gaze);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "radar_stack_update: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (gaze.human)
        {
            float az = aim_azimuth_deg(&gaze);
            gaze_xy_from_az_elev(az, gaze.elevation_norm, &tgt_x, &tgt_y);
            tgt_r = radius_from_distance_mm(gaze.primary.distance_mm);
        }
        else
        {
            tgt_x = center_x;
            tgt_y = center_y;
            tgt_r = (float)CFG_DOT_RADIUS_MIN;
            reset_azimuth_coast();
            if (last_human)
            {
                ESP_LOGI(TAG, "$LOST t=%lu", (unsigned long)esp_log_timestamp());
                log_skip = 0;
            }
        }
        last_human = gaze.human;

        cur_x += (tgt_x - cur_x) * CFG_SMOOTH_H;
        cur_y += (tgt_y - cur_y) * CFG_SMOOTH_V;
        cur_r += (tgt_r - cur_r) * CFG_SMOOTH_R;

        int eye_x = (int)lroundf(cur_x);
        int eye_y = (int)lroundf(cur_y);
        int eye_r = (int)lroundf(cur_r);

        if (++log_skip >= 10)
        {
            log_skip = 0;
            log_frame(&gaze, eye_x, eye_y);
        }

        display_set_dot(eye_x, eye_y, eye_r);

        /* Update perf stats */
        {
            int64_t now_us = esp_timer_get_time();
            int64_t frame_us = now_us - s_perf_frame_begin_us;
            s_perf_frame_count++;
            s_perf_frame_sum_us += frame_us;
            if (frame_us < s_perf_frame_min_us)
                s_perf_frame_min_us = frame_us;
            if (frame_us > s_perf_frame_max_us)
                s_perf_frame_max_us = frame_us;

            int64_t elapsed_us = now_us - s_perf_start_us;
            if (elapsed_us >= PERF_INTERVAL_US && s_perf_frame_count > 0)
            {
                float fps = (float)s_perf_frame_count / ((float)elapsed_us * 1e-6f);
                int64_t avg_us = s_perf_frame_sum_us / (int64_t)s_perf_frame_count;
                ESP_LOGI(TAG,
                         "$PERF fps=%.1f frame_us=%lld/%lld/%lld"
                         "(min/avg/max) frames=%lu",
                         fps, (long long)s_perf_frame_min_us, (long long)avg_us,
                         (long long)s_perf_frame_max_us, (unsigned long)s_perf_frame_count);

                /* Reset for next period */
                s_perf_start_us = now_us;
                s_perf_frame_count = 0;
                s_perf_frame_min_us = INT64_MAX;
                s_perf_frame_max_us = 0;
                s_perf_frame_sum_us = 0;
            }
        }
    }
}
