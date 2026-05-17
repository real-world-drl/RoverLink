// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Wraps QMI8658 (accel+gyro) and AK09918 (mag). One call returns the
// combined latest sample. The mag chip runs at its own 100 Hz; mag fields
// are updated on each call only when DRDY is set, otherwise they retain
// the previous reading. `mag_fresh` lets the caller / host distinguish.
esp_err_t imu_init(void);

esp_err_t imu_read(float acc_ms2[3], float gyr_rads[3], float mag_ut[3],
                   float *temp_c, bool *mag_fresh);
