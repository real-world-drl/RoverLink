// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "qmi8658.h"
#include "i2c_bus.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "qmi8658";

#define QMI_ADDR        0x6B

#define REG_WHO_AM_I    0x00
#define REG_CTRL1       0x02
#define REG_CTRL2       0x03   // accel range + ODR
#define REG_CTRL3       0x04   // gyro range + ODR
#define REG_CTRL5       0x06   // LPF
#define REG_CTRL7       0x08   // enable
#define REG_TEMP_L      0x33
#define REG_ACC_X_L     0x35   // 12 bytes: AccX..AccZ then GyrX..GyrZ

// CTRL2 = (range<<4) | odr.  ±8 g  = 0x20, ODR 1000Hz = 0x03  ->  0x23
// CTRL3 = (range<<4) | odr.  ±2048 dps = 0x70, ODR 1000Hz = 0x03 -> 0x73
#define ACC_CTRL2       0x23
#define GYR_CTRL3       0x73

#define ACC_LSB_PER_G   (1 << 12)         // ±8 g -> 4096 LSB/g
#define GYR_LSB_PER_DPS (16)              // ±2048 dps -> 16 LSB/dps
#define G_TO_MS2        9.80665f
#define DPS_TO_RADS     (3.14159265358979323846f / 180.0f)

esp_err_t qmi8658_init(void) {
    uint8_t who = 0;
    esp_err_t err = i2c_bus_read_reg(QMI_ADDR, REG_WHO_AM_I, &who);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (who != 0x05) {
        ESP_LOGW(TAG, "unexpected WHO_AM_I 0x%02X (expected 0x05) — continuing", who);
    }

    // CTRL1: address auto-increment, internal-2 MHz osc, INT off.
    i2c_bus_write_reg(QMI_ADDR, REG_CTRL1, 0x40);
    i2c_bus_write_reg(QMI_ADDR, REG_CTRL2, ACC_CTRL2);
    i2c_bus_write_reg(QMI_ADDR, REG_CTRL3, GYR_CTRL3);
    i2c_bus_write_reg(QMI_ADDR, REG_CTRL5, 0x00);   // no LPF (raw samples)
    i2c_bus_write_reg(QMI_ADDR, REG_CTRL7, 0x03);   // enable accel + gyro
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "ready at 0x%02X (±8g, ±2048dps, 1kHz ODR)", QMI_ADDR);
    return ESP_OK;
}

esp_err_t qmi8658_read(float acc_ms2[3], float gyr_rads[3], float *temp_c) {
    uint8_t buf[12];
    esp_err_t err = i2c_bus_read_regs(QMI_ADDR, REG_ACC_X_L, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    const int16_t ax_raw = (int16_t)(buf[0]  | (buf[1]  << 8));
    const int16_t ay_raw = (int16_t)(buf[2]  | (buf[3]  << 8));
    const int16_t az_raw = (int16_t)(buf[4]  | (buf[5]  << 8));
    const int16_t gx_raw = (int16_t)(buf[6]  | (buf[7]  << 8));
    const int16_t gy_raw = (int16_t)(buf[8]  | (buf[9]  << 8));
    const int16_t gz_raw = (int16_t)(buf[10] | (buf[11] << 8));

    acc_ms2[0] = (float)ax_raw / ACC_LSB_PER_G * G_TO_MS2;
    acc_ms2[1] = (float)ay_raw / ACC_LSB_PER_G * G_TO_MS2;
    acc_ms2[2] = (float)az_raw / ACC_LSB_PER_G * G_TO_MS2;

    gyr_rads[0] = (float)gx_raw / GYR_LSB_PER_DPS * DPS_TO_RADS;
    gyr_rads[1] = (float)gy_raw / GYR_LSB_PER_DPS * DPS_TO_RADS;
    gyr_rads[2] = (float)gz_raw / GYR_LSB_PER_DPS * DPS_TO_RADS;

    if (temp_c) {
        uint8_t tbuf[2];
        if (i2c_bus_read_regs(QMI_ADDR, REG_TEMP_L, tbuf, 2) == ESP_OK) {
            // Datasheet: temp = signed_int16 / 256 + 0 °C  (no offset; 0=0°C).
            const int16_t t_raw = (int16_t)(tbuf[0] | (tbuf[1] << 8));
            *temp_c = (float)t_raw / 256.0f;
        } else {
            *temp_c = NAN;
        }
    }
    return ESP_OK;
}
