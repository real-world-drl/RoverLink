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

#define UGV_STATUS_ONLINE    "online"
#define UGV_STATUS_OFFLINE   "offline"

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
    float deadband;               // |out| < deadband -> 0
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
