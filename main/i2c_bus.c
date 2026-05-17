// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "i2c_bus.h"

#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c";

#define I2C_PORT       I2C_NUM_0
#define I2C_PIN_SDA    32
#define I2C_PIN_SCL    33
#define I2C_FREQ_HZ    400000
#define I2C_TIMEOUT_MS 100

esp_err_t i2c_bus_init(void) {
    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_PIN_SDA,
        .scl_io_num       = I2C_PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "I2C master on SDA=%d SCL=%d @ %d Hz",
             I2C_PIN_SDA, I2C_PIN_SCL, I2C_FREQ_HZ);
    return ESP_OK;
}

esp_err_t i2c_bus_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, dev_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_bus_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(I2C_PORT, dev_addr, &reg, 1, val, 1,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_bus_read_regs(uint8_t dev_addr, uint8_t reg,
                            uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, dev_addr, &reg, 1, buf, len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_bus_write_word(uint8_t dev_addr, uint8_t reg, uint16_t val) {
    // INA219 expects big-endian register words.
    const uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_write_to_device(I2C_PORT, dev_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_bus_read_word(uint8_t dev_addr, uint8_t reg, uint16_t *val) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, dev_addr, &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}
