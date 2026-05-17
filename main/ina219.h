// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include "esp_err.h"

// Minimal INA219 driver matching the stock Waveshare General Driver config:
//   I2C address 0x42, 0.01 Ω shunt, PGA ±320 mV, bus range 16 V, 9-bit ADC.
//   Current LSB = 100 µA  =>  CAL = 4096.
//   Power  LSB = 20 * Current LSB = 2 mW.
//
// Reading: bus voltage in V, current in A. Sign of current follows shunt
// polarity; flip a wire (or post-process on host) if the sign is wrong.
esp_err_t ina219_init(void);
esp_err_t ina219_read(float *voltage_v, float *current_a);
