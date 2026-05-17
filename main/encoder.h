// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stdint.h>
#include "esp_err.h"

// Initializes PCNT units for both wheel encoders.
// 2x quadrature decoding to match stock firmware's PPR figure.
esp_err_t encoder_init(void);

// Cumulative ticks since boot. Sign convention: positive = forward rotation
// of that wheel (after CONFIG_UGV_ENCODER_*_INVERT is applied). Safe from
// any task context.
int32_t encoder_left_ticks(void);
int32_t encoder_right_ticks(void);

// Diagnostic: logs raw GPIO levels of all four encoder pins plus the raw
// PCNT counts. Call periodically (e.g. once a second) to see whether the
// encoder lines are actually toggling.
void    encoder_debug_log(void);
