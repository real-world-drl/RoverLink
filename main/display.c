// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "display.h"

#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "comms.h"
#include "ina219.h"
#include "odometry.h"
#include "oled.h"
#include "ugv_packets.h"

static const char *TAG = "display";

esp_err_t display_init(void) {
    return oled_init();
}

static inline char sign_of(float v) { return v < 0 ? '-' : '+'; }

void display_task(void *arg) {
    (void)arg;

    ugv_cmd_display_t host_pkt = {0};
    bool seen_host = false;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_UGV_DISPLAY_HZ);
    const int64_t   timeout_us = (int64_t)CONFIG_UGV_DISPLAY_HOST_TIMEOUT_MS * 1000;
    TickType_t next = xTaskGetTickCount();

    char line[24];

    for (;;) {
        vTaskDelayUntil(&next, period);

        // Drain any pending host update into the cached copy.
        ugv_cmd_display_t fresh;
        if (comms_take_cmd_display(&fresh)) {
            host_pkt = fresh;
            seen_host = true;
        }

        const int64_t now  = esp_timer_get_time();
        const int64_t recv = comms_cmd_display_recv_us();
        const bool use_host =
            seen_host && recv != 0 && (now - recv) < timeout_us;

        float x, y, yaw, pitch, roll;
        if (use_host) {
            x = host_pkt.x;
            y = host_pkt.y;
            yaw   = host_pkt.yaw;
            pitch = host_pkt.pitch;
            roll  = host_pkt.roll;
        } else {
            odometry_get(&x, &y, &yaw, &pitch, &roll);
        }

        const float yaw_d   = yaw   * 180.0f / (float)M_PI;
        const float pitch_d = pitch * 180.0f / (float)M_PI;
        const float roll_d  = roll  * 180.0f / (float)M_PI;

        oled_clear();
        // 128x32 panel = 4 lines × 21 char cells. Compact layout to fit
        // x, y, YPR in 4 lines with a source indicator pinned top-right.
        oled_text(0, 20, use_host ? "H" : "L");

        snprintf(line, sizeof(line), "x:%c%5.3f y:%c%5.3f",
                 sign_of(x), fabsf(x), sign_of(y), fabsf(y));
        oled_text(0, 0, line);

        snprintf(line, sizeof(line), "Y:%c%5.1f  P:%c%5.1f",
                 sign_of(yaw_d), fabsf(yaw_d), sign_of(pitch_d), fabsf(pitch_d));
        oled_text(1, 0, line);

        snprintf(line, sizeof(line), "R:%c%5.1f d",
                 sign_of(roll_d), fabsf(roll_d));
        oled_text(2, 0, line);

#ifdef CONFIG_UGV_ENABLE_INA219
        float v_bat = NAN, i_bat = NAN;
        if (ina219_read(&v_bat, &i_bat) == ESP_OK && !isnan(v_bat)) {
            snprintf(line, sizeof(line), "V: %5.2f", v_bat);
        } else {
            snprintf(line, sizeof(line), "V: -----");
        }
        oled_text(3, 0, line);
#endif

        esp_err_t err = oled_flush();
        if (err != ESP_OK) ESP_LOGW(TAG, "flush failed: %s", esp_err_to_name(err));
    }
}
