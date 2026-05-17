// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include "esp_err.h"

// QMI8658 6-axis IMU driver — minimal.
// Defaults to ±8 g accel, ±2048 dps gyro, 1 kHz ODR for both, no built-in LPF.
// Caller polls qmi8658_read() at its own rate (typically 100–500 Hz).
//
// Output:
//   acc_ms2[0..2] — accelerometer in m/s² (gravity + linear)
//   gyr_rads[0..2] — gyroscope in rad/s
//
// Axis convention follows the chip silkscreen; verify orientation on the
// robot at bring-up and apply any sign flips / swaps on the host.
esp_err_t qmi8658_init(void);
esp_err_t qmi8658_read(float acc_ms2[3], float gyr_rads[3], float *temp_c);
