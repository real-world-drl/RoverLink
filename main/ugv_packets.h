// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

// Wire format between firmware and host. Little-endian. Share this header
// with the host application to avoid drift.
//
// Topic conventions (with <id> = CONFIG_UGV_ROBOT_ID):
//   ugv/<id>/v1/cmd/vel      host -> bot,  QoS 1, not retained
//   ugv/<id>/v1/cmd/pid      host -> bot,  QoS 1, retained
//   ugv/<id>/v1/tel/wheel    bot  -> host, QoS 0
//   ugv/<id>/v1/tel/imu      bot  -> host, QoS 0
//   ugv/<id>/v1/tel/battery  bot  -> host, QoS 0
//   ugv/<id>/v1/status       bot  -> host, QoS 1, retained ("online"/"offline" via LWT)

#include <stdint.h>

#define UGV_TOPIC_VERSION    "v1"
#define UGV_TOPIC_CMD_VEL     "cmd/vel"
#define UGV_TOPIC_CMD_PID     "cmd/pid"
#define UGV_TOPIC_CMD_DISPLAY "cmd/display"
#define UGV_TOPIC_TEL_WHEEL  "tel/wheel"
#define UGV_TOPIC_TEL_IMU    "tel/imu"
#define UGV_TOPIC_TEL_BATT   "tel/battery"
#define UGV_TOPIC_STATUS     "status"
// Control-plane only: payload is a plain UTF-8 URL string (not a packed
// struct), since OTA is a one-shot action, not part of the binary wire
// telemetry. MQTT transport only — OTA needs WiFi regardless.
#define UGV_TOPIC_CMD_OTA    "cmd/ota"
// OTA progress/result, published as a plain UTF-8 string (human-readable,
// retained so the last outcome is visible to a late subscriber). The
// console is disabled in this build, so this is how OTA failures surface.
#define UGV_TOPIC_TEL_OTA    "tel/ota"

#define UGV_STATUS_ONLINE    "online"
#define UGV_STATUS_OFFLINE   "offline"

// --- UART transport framing -------------------------------------------------
// On the UART link, packets are framed as:
//   [UGV_UART_SYNC] [type:1] [len:1] [payload:len] [crc8:1]
// where crc8 covers (type, len, payload) using poly 0x07, init 0x00.
// `type` identifies which packed struct the payload is. The host and
// firmware both use the same struct definitions below as the payload.
#define UGV_UART_SYNC 0xA5

typedef enum {
    UGV_PKT_CMD_VEL     = 0x01,
    UGV_PKT_CMD_PID     = 0x02,
    UGV_PKT_CMD_DISPLAY = 0x03,
    UGV_PKT_TEL_WHEEL   = 0x10,
    UGV_PKT_TEL_IMU     = 0x11,
    UGV_PKT_TEL_BATT    = 0x12,
    UGV_PKT_STATUS      = 0x20,
} ugv_pkt_type_t;

typedef struct __attribute__((packed)) {
    uint64_t host_timestamp_us;   // host clock, used by host for latency stats
    float    linear_x;            // m/s, +forward
    float    angular_z;           // rad/s, +CCW (viewed from above)
} ugv_cmd_vel_t;                  // 16 B

typedef struct __attribute__((packed)) {
    float kp;
    float ki;
    float kd;
    float output_clamp;           // ± PWM counts
    float deadband;               // reserved (v1): no longer applied —
                                  // closed-loop uses CONFIG_UGV_MIN_DRIVE_PWM
} ugv_cmd_pid_t;                  // 20 B

// Host-published pose for the OLED display. The firmware also computes its
// own (crude) estimate from wheels+IMU; if a fresh cmd/display packet hasn't
// arrived within CONFIG_UGV_DISPLAY_HOST_TIMEOUT_MS the display falls back
// to the firmware estimate.
typedef struct __attribute__((packed)) {
    uint64_t host_timestamp_us;
    float    x;                   // m
    float    y;                   // m
    float    yaw;                 // rad
    float    pitch;               // rad
    float    roll;                // rad
} ugv_cmd_display_t;              // 28 B

typedef struct __attribute__((packed)) {
    uint64_t device_timestamp_us; // esp_timer_get_time()
    uint32_t seq;                 // monotonic
    int32_t  left_ticks;          // cumulative encoder ticks (signed)
    int32_t  right_ticks;
    float    left_velocity_mps;
    float    right_velocity_mps;
    float    left_setpoint_mps;
    float    right_setpoint_mps;
} ugv_wheel_telem_t;              // 36 B

typedef struct __attribute__((packed)) {
    uint64_t device_timestamp_us;
    float    voltage_v;
    float    current_a;
} ugv_battery_telem_t;            // 16 B

typedef struct __attribute__((packed)) {
    uint64_t device_timestamp_us;
    uint32_t seq;
    float    ax, ay, az;          // m/s² (gravity + linear)
    float    gx, gy, gz;          // rad/s
    float    mx, my, mz;          // µT (latest fresh reading; may repeat)
    float    temp_c;
    uint8_t  mag_fresh;           // 1 if mag was updated this packet, else 0
    uint8_t  _pad[3];
} ugv_imu_telem_t;                // 56 B

// Compile-time guards: if anyone reorders / adds a field, the size comment
// above is wrong and the host parser will misalign. These _Static_asserts
// fail the build instead of silently shipping a broken contract.
_Static_assert(sizeof(ugv_cmd_vel_t)       == 16, "ugv_cmd_vel_t size drift");
_Static_assert(sizeof(ugv_cmd_pid_t)       == 20, "ugv_cmd_pid_t size drift");
_Static_assert(sizeof(ugv_cmd_display_t)   == 28, "ugv_cmd_display_t size drift");
_Static_assert(sizeof(ugv_wheel_telem_t)   == 36, "ugv_wheel_telem_t size drift");
_Static_assert(sizeof(ugv_battery_telem_t) == 16, "ugv_battery_telem_t size drift");
_Static_assert(sizeof(ugv_imu_telem_t)     == 56, "ugv_imu_telem_t size drift");
