// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "comms.h"
#include "display.h"
#include "encoder.h"
#include "i2c_bus.h"
#include "imu.h"
#include "ina219.h"
#include "kinematics.h"
#include "motor.h"
#include "odometry.h"
#include "pid.h"
#include "uart_link.h"
#include "ugv_packets.h"

static const char *TAG = "ugv";

#define HEARTBEAT_TIMEOUT_US (CONFIG_UGV_HEARTBEAT_TIMEOUT_MS * 1000LL)

// Snapshot pushed from control_task to telemetry_task. Queue capacity 1
// with xQueueOverwrite — telemetry always sees the latest tick.
typedef struct {
    int64_t  t_us;
    int32_t  left_ticks,  right_ticks;
    float    left_vel,    right_vel;
    float    left_set,    right_set;
} ctrl_snapshot_t;

static QueueHandle_t s_snapshot_q;
static kinematics_t  s_kin;
static ugv_pid_t s_pid_l, s_pid_r;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// --- control loop -------------------------------------------------------

static void control_task(void *arg) {
    (void)arg;

    int32_t last_left_ticks  = encoder_left_ticks();
    int32_t last_right_ticks = encoder_right_ticks();
    int64_t last_us = esp_timer_get_time();
    int64_t last_cmd_us = 0;

    float set_lin = 0.0f, set_ang = 0.0f;
    float v_set_l = 0.0f, v_set_r = 0.0f;

    const float max_lin = CONFIG_UGV_MAX_LINEAR_MPS_X100 / 100.0f;
    const float max_ang = CONFIG_UGV_MAX_ANGULAR_RPS_X100 / 100.0f;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_UGV_CONTROL_HZ);
    TickType_t next_wake = xTaskGetTickCount();
    uint32_t loop_count = 0;
    const uint32_t telem_div =
        (CONFIG_UGV_CONTROL_HZ + CONFIG_UGV_TELEMETRY_HZ / 2) / CONFIG_UGV_TELEMETRY_HZ;

    for (;;) {
        vTaskDelayUntil(&next_wake, period);
        const int64_t now_us = esp_timer_get_time();

        ugv_cmd_pid_t new_pid;
        if (comms_take_cmd_pid(&new_pid)) {
            pid_set_tunings(&s_pid_l, new_pid.kp, new_pid.ki, new_pid.kd);
            pid_set_tunings(&s_pid_r, new_pid.kp, new_pid.ki, new_pid.kd);
            pid_set_output_limits(&s_pid_l, -new_pid.output_clamp, new_pid.output_clamp);
            pid_set_output_limits(&s_pid_r, -new_pid.output_clamp, new_pid.output_clamp);
            pid_set_deadband(&s_pid_l, new_pid.deadband);
            pid_set_deadband(&s_pid_r, new_pid.deadband);
            ESP_LOGI(TAG, "PID updated: kp=%.3f ki=%.3f kd=%.3f clamp=%.0f db=%.0f",
                     new_pid.kp, new_pid.ki, new_pid.kd,
                     new_pid.output_clamp, new_pid.deadband);
        }

        ugv_cmd_vel_t new_vel;
        if (comms_take_cmd_vel(&new_vel)) {
            set_lin = clampf(new_vel.linear_x,  -max_lin, max_lin);
            set_ang = clampf(new_vel.angular_z, -max_ang, max_ang);
            last_cmd_us = now_us;
        }

        const bool stale = (last_cmd_us == 0) ||
                           (now_us - last_cmd_us > HEARTBEAT_TIMEOUT_US);
        if (stale) { set_lin = 0.0f; set_ang = 0.0f; }

        kinematics_cmd_vel_to_wheels(&s_kin, set_lin, set_ang, &v_set_l, &v_set_r);

        const int32_t left_ticks  = encoder_left_ticks();
        const int32_t right_ticks = encoder_right_ticks();
        const float dt = (float)(now_us - last_us) * 1e-6f;
        float v_meas_l = 0.0f, v_meas_r = 0.0f;
        if (dt > 0.0f) {
            v_meas_l = kinematics_ticks_to_meters(&s_kin,
                          left_ticks  - last_left_ticks ) / dt;
            v_meas_r = kinematics_ticks_to_meters(&s_kin,
                          right_ticks - last_right_ticks) / dt;
        }
        last_left_ticks  = left_ticks;
        last_right_ticks = right_ticks;
        last_us          = now_us;

        float out_l, out_r;
#if CONFIG_UGV_OPEN_LOOP
        // No encoder feedback — map setpoint directly to PWM, clamp,
        // apply the same deadband the PID uses to avoid sub-stall hum.
        const float scale = CONFIG_UGV_OPEN_LOOP_PWM_PER_MPS_X10 / 10.0f;
        const float clamp = (float)CONFIG_UGV_PID_OUTPUT_CLAMP;
        const float deadb = (float)CONFIG_UGV_PID_DEADBAND;
        out_l = clampf(v_set_l * scale, -clamp, clamp);
        out_r = clampf(v_set_r * scale, -clamp, clamp);
        if (fabsf(out_l) < deadb) out_l = 0.0f;
        if (fabsf(out_r) < deadb) out_r = 0.0f;
#else
        out_l = pid_compute(&s_pid_l, v_set_l, v_meas_l, now_us);
        out_r = pid_compute(&s_pid_r, v_set_r, v_meas_r, now_us);
#endif

        // Feed wheel velocities into the firmware odometry (display only).
        // Open-loop: use setpoints as a stand-in for the measurement we don't
        // have, so x/y still trend (with whatever error the bot accumulates).
        if (dt > 0.0f) {
#if CONFIG_UGV_OPEN_LOOP
            odometry_update_wheels(v_set_l, v_set_r, dt);
#else
            odometry_update_wheels(v_meas_l, v_meas_r, dt);
#endif
        }

        if (stale) {
            motor_stop_all();
            pid_reset(&s_pid_l);
            pid_reset(&s_pid_r);
        } else {
            motor_set_left_pwm ((int16_t)lrintf(out_l));
            motor_set_right_pwm((int16_t)lrintf(out_r));
        }

        if ((loop_count++ % telem_div) == 0) {
            const ctrl_snapshot_t snap = {
                .t_us = now_us,
                .left_ticks  = left_ticks,
                .right_ticks = right_ticks,
                .left_vel  = v_meas_l, .right_vel = v_meas_r,
                .left_set  = v_set_l,  .right_set = v_set_r,
            };
            xQueueOverwrite(s_snapshot_q, &snap);
        }
    }
}

// --- telemetry ----------------------------------------------------------

static void telemetry_task(void *arg) {
    (void)arg;

    uint32_t seq = 0;
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_UGV_TELEMETRY_HZ);
    const uint32_t batt_div =
        (CONFIG_UGV_TELEMETRY_HZ + CONFIG_UGV_BATTERY_HZ / 2) / CONFIG_UGV_BATTERY_HZ;
    uint32_t tick = 0;

    for (;;) {
        ctrl_snapshot_t snap;
        if (xQueueReceive(s_snapshot_q, &snap, period) == pdTRUE) {
            const ugv_wheel_telem_t wt = {
                .device_timestamp_us = (uint64_t)snap.t_us,
                .seq = seq++,
                .left_ticks  = snap.left_ticks,
                .right_ticks = snap.right_ticks,
                .left_velocity_mps  = snap.left_vel,
                .right_velocity_mps = snap.right_vel,
                .left_setpoint_mps  = snap.left_set,
                .right_setpoint_mps = snap.right_set,
            };
#ifdef CONFIG_UGV_ENABLE_MQTT
            comms_publish_wheel(&wt);
#endif
#ifdef CONFIG_UGV_ENABLE_UART_LINK
            uart_link_publish_wheel(&wt);
#endif
        }

        if ((tick % batt_div) == 0) {
            float v = NAN, i = NAN;
#ifdef CONFIG_UGV_ENABLE_INA219
            ina219_read(&v, &i);
#endif
            const ugv_battery_telem_t bt = {
                .device_timestamp_us = (uint64_t)esp_timer_get_time(),
                .voltage_v = v,
                .current_a = i,
            };
#ifdef CONFIG_UGV_ENABLE_MQTT
            comms_publish_battery(&bt);
#endif
#ifdef CONFIG_UGV_ENABLE_UART_LINK
            uart_link_publish_battery(&bt);
#endif
        }
        // 1 Hz encoder GPIO/PCNT diagnostic.
        if ((tick % CONFIG_UGV_TELEMETRY_HZ) == 0) {
            encoder_debug_log();
        }
        tick++;
    }
}

// --- IMU task -----------------------------------------------------------

#ifdef CONFIG_UGV_ENABLE_IMU
// Body-frame axis signs for the display-side odometry. Kconfig bools emit
// no macro when unset; promote them to ±1.0 here.
#ifdef CONFIG_UGV_IMU_BODY_INVERT_X
#  define IMU_BODY_SX (-1.0f)
#else
#  define IMU_BODY_SX (+1.0f)
#endif
#ifdef CONFIG_UGV_IMU_BODY_INVERT_Y
#  define IMU_BODY_SY (-1.0f)
#else
#  define IMU_BODY_SY (+1.0f)
#endif
#ifdef CONFIG_UGV_IMU_BODY_INVERT_Z
#  define IMU_BODY_SZ (-1.0f)
#else
#  define IMU_BODY_SZ (+1.0f)
#endif

static void imu_task(void *arg) {
    (void)arg;

    uint32_t seq = 0;
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_UGV_IMU_HZ);
    const float dt = 1.0f / (float)CONFIG_UGV_IMU_HZ;
    TickType_t next_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next_wake, period);

        float acc[3], gyr[3], mag[3], temp_c = NAN;
        bool mag_fresh = false;
        if (imu_read(acc, gyr, mag, &temp_c, &mag_fresh) != ESP_OK) continue;

        // Feed body-frame-corrected values into firmware odometry. Raw chip
        // values still get published verbatim below — host does its own
        // calibration on the published telemetry.
        odometry_update_imu(IMU_BODY_SX * acc[0],
                            IMU_BODY_SY * acc[1],
                            IMU_BODY_SZ * acc[2],
                            IMU_BODY_SX * gyr[0],
                            IMU_BODY_SY * gyr[1],
                            IMU_BODY_SZ * gyr[2], dt);

        const ugv_imu_telem_t pkt = {
            .device_timestamp_us = (uint64_t)esp_timer_get_time(),
            .seq = seq++,
            .ax = acc[0], .ay = acc[1], .az = acc[2],
            .gx = gyr[0], .gy = gyr[1], .gz = gyr[2],
            .mx = mag[0], .my = mag[1], .mz = mag[2],
            .temp_c = temp_c,
            .mag_fresh = mag_fresh ? 1 : 0,
        };
#ifdef CONFIG_UGV_ENABLE_MQTT
        comms_publish_imu(&pkt);
#endif
#ifdef CONFIG_UGV_ENABLE_UART_LINK
        uart_link_publish_imu(&pkt);
#endif
    }
}
#endif

// --- entry --------------------------------------------------------------

void app_main(void) {
    ESP_LOGI(TAG, "UGV firmware boot");

    // Shared command queues — must exist before any transport tries to push.
    comms_queues_init();

#ifdef CONFIG_UGV_ENABLE_MQTT
    if (comms_init() != ESP_OK) {
        ESP_LOGE(TAG, "MQTT comms init failed — continuing");
    }
#endif
#ifdef CONFIG_UGV_ENABLE_UART_LINK
    if (uart_link_init() != ESP_OK) {
        ESP_LOGE(TAG, "UART link init failed — continuing");
    }
#endif

    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(encoder_init());

#if defined(CONFIG_UGV_ENABLE_INA219) || defined(CONFIG_UGV_ENABLE_IMU)
    ESP_ERROR_CHECK(i2c_bus_init());
#endif
#ifdef CONFIG_UGV_ENABLE_INA219
    if (ina219_init() != ESP_OK) {
        ESP_LOGW(TAG, "INA219 not present — battery telemetry will be NaN");
    }
#endif
#ifdef CONFIG_UGV_ENABLE_IMU
    if (imu_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed — IMU telemetry disabled");
    }
#endif
#ifdef CONFIG_UGV_ENABLE_DISPLAY
    if (display_init() != ESP_OK) {
        ESP_LOGW(TAG, "OLED init failed — display disabled");
    }
#endif

    odometry_init(CONFIG_UGV_TRACK_WIDTH_MM / 1000.0f);

    kinematics_init(&s_kin,
                    CONFIG_UGV_WHEEL_DIAMETER_MM / 1000.0f,
                    CONFIG_UGV_TRACK_WIDTH_MM   / 1000.0f,
                    CONFIG_UGV_ENCODER_PPR);

    const float kp = CONFIG_UGV_PID_KP_X1000 / 1000.0f;
    const float ki = CONFIG_UGV_PID_KI_X1000 / 1000.0f;
    const float kd = CONFIG_UGV_PID_KD_X1000 / 1000.0f;
    pid_init(&s_pid_l, kp, ki, kd,
             -CONFIG_UGV_PID_OUTPUT_CLAMP, CONFIG_UGV_PID_OUTPUT_CLAMP,
             CONFIG_UGV_PID_DEADBAND);
    pid_init(&s_pid_r, kp, ki, kd,
             -CONFIG_UGV_PID_OUTPUT_CLAMP, CONFIG_UGV_PID_OUTPUT_CLAMP,
             CONFIG_UGV_PID_DEADBAND);
    ESP_LOGI(TAG, "kinematics: wheel=%.3fm track=%.3fm ppr=%d  m/tick=%.6f",
             s_kin.wheel_diameter_m, s_kin.track_width_m,
             s_kin.encoder_ppr, s_kin.meters_per_tick);
    ESP_LOGI(TAG, "PID start: kp=%.3f ki=%.3f kd=%.3f clamp=±%d db=%d hb=%dms",
             kp, ki, kd,
             CONFIG_UGV_PID_OUTPUT_CLAMP, CONFIG_UGV_PID_DEADBAND,
             CONFIG_UGV_HEARTBEAT_TIMEOUT_MS);

    s_snapshot_q = xQueueCreate(1, sizeof(ctrl_snapshot_t));

    xTaskCreatePinnedToCore(control_task,   "control",   4096, NULL, 12, NULL, 1);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL,  8, NULL, 0);
#ifdef CONFIG_UGV_ENABLE_IMU
    xTaskCreatePinnedToCore(imu_task,       "imu",       4096, NULL, 10, NULL, 0);
#endif
#ifdef CONFIG_UGV_ENABLE_DISPLAY
    xTaskCreatePinnedToCore(display_task,   "display",   4096, NULL,  5, NULL, 0);
#endif
}
