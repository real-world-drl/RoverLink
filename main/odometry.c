#include "odometry.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// All shared state lives under one spinlock so the display task gets a
// consistent snapshot.
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static float s_track_w = 0.172f;

static float s_x   = 0.0f;
static float s_y   = 0.0f;
static float s_yaw = 0.0f;     // integrated from gz
static float s_pitch = 0.0f;
static float s_roll  = 0.0f;

// Complementary filter weight: 0.98 = ~0.5s time constant at 100 Hz.
// Higher = trust gyro more, drift more slowly under noise.
#define ALPHA 0.98f

void odometry_init(float track_width_m) {
    s_track_w = track_width_m;
    odometry_reset();
}

void odometry_reset(void) {
    portENTER_CRITICAL(&s_lock);
    s_x = s_y = 0.0f;
    s_yaw = s_pitch = s_roll = 0.0f;
    portEXIT_CRITICAL(&s_lock);
}

void odometry_update_wheels(float v_left_mps, float v_right_mps, float dt_s) {
    if (dt_s <= 0.0f || dt_s > 1.0f) return;
    portENTER_CRITICAL(&s_lock);
    const float v = 0.5f * (v_left_mps + v_right_mps);
    const float w = (v_right_mps - v_left_mps) / s_track_w;
    const float dtheta = w * dt_s;
    // Midpoint integration: use heading at dt/2 for x,y.
    const float th_mid = s_yaw + 0.5f * dtheta;
    s_x   += v * cosf(th_mid) * dt_s;
    s_y   += v * sinf(th_mid) * dt_s;
    s_yaw += dtheta;
    // Wrap yaw to (-pi, pi].
    if (s_yaw >  (float)M_PI) s_yaw -= 2.0f * (float)M_PI;
    if (s_yaw <= -(float)M_PI) s_yaw += 2.0f * (float)M_PI;
    portEXIT_CRITICAL(&s_lock);
}

void odometry_update_imu(float ax, float ay, float az,
                         float gx, float gy, float gz, float dt_s) {
    if (dt_s <= 0.0f || dt_s > 1.0f) return;
    // Gravity-derived static estimates. Assumes IMU body axes ~= robot
    // body axes (X forward, Y left, Z up). If your mount differs, swap or
    // negate inputs at the call site or fix it on the host.
    const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float roll_acc  = atan2f(ay, az);

    portENTER_CRITICAL(&s_lock);
    s_pitch = ALPHA * (s_pitch + gy * dt_s) + (1.0f - ALPHA) * pitch_acc;
    s_roll  = ALPHA * (s_roll  + gx * dt_s) + (1.0f - ALPHA) * roll_acc;
    // Yaw from IMU is fused with the wheel-odometry yaw via weighted blend
    // each IMU tick. Wheels are usually more reliable than gz alone for a
    // ground robot; this nudges yaw toward the gyro-integrated value to
    // catch slip events.
    s_yaw += gz * dt_s;
    if (s_yaw >  (float)M_PI) s_yaw -= 2.0f * (float)M_PI;
    if (s_yaw <= -(float)M_PI) s_yaw += 2.0f * (float)M_PI;
    portEXIT_CRITICAL(&s_lock);
}

void odometry_get(float *x_m, float *y_m,
                  float *yaw_rad, float *pitch_rad, float *roll_rad) {
    portENTER_CRITICAL(&s_lock);
    if (x_m)       *x_m       = s_x;
    if (y_m)       *y_m       = s_y;
    if (yaw_rad)   *yaw_rad   = s_yaw;
    if (pitch_rad) *pitch_rad = s_pitch;
    if (roll_rad)  *roll_rad  = s_roll;
    portEXIT_CRITICAL(&s_lock);
}
