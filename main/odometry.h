// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stdint.h>

// Firmware-side pose estimate for the OLED display.
// Wheel-odometry x/y is integrated from per-tick velocities; YPR comes from
// a complementary filter on IMU gyro + accelerometer (mag not used to keep
// it self-contained — yaw therefore drifts).
//
// This is *not* authoritative. The host runs its own (better) pose
// estimator from the raw telemetry. These values exist purely so the OLED
// has something sensible to show when the host hasn't sent a display
// update recently.

void odometry_init(float track_width_m);

// Called from control_task each tick with the latest measured wheel speeds.
void odometry_update_wheels(float v_left_mps, float v_right_mps, float dt_s);

// Called from imu_task each sample with accel (m/s²) and gyro (rad/s).
void odometry_update_imu(float ax, float ay, float az,
                         float gx, float gy, float gz, float dt_s);

// Snapshot — non-blocking, takes a brief spinlock.
void odometry_get(float *x_m, float *y_m,
                  float *yaw_rad, float *pitch_rad, float *roll_rad);

// Reset everything to zero. Handy for an "origin = here" button if you
// ever wire one up.
void odometry_reset(void);
