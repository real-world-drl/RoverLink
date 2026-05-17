// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Shared I2C master on GPIO 32 (SDA) / 33 (SCL) at 400 kHz.
// Pins match the Waveshare General Driver board layout.
// The legacy IDF i2c driver locks the port internally, so concurrent
// callers from different tasks are safe.
esp_err_t i2c_bus_init(void);

esp_err_t i2c_bus_write_reg (uint8_t dev_addr, uint8_t reg, uint8_t  val);
esp_err_t i2c_bus_read_reg  (uint8_t dev_addr, uint8_t reg, uint8_t *val);
esp_err_t i2c_bus_read_regs (uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len);
esp_err_t i2c_bus_write_word(uint8_t dev_addr, uint8_t reg, uint16_t val); // big-endian wire order
esp_err_t i2c_bus_read_word (uint8_t dev_addr, uint8_t reg, uint16_t *val);
