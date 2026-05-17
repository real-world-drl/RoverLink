// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "ina219.h"
#include "i2c_bus.h"

#include "esp_log.h"

static const char *TAG = "ina219";

#define INA219_ADDR           0x42

#define INA219_REG_CONFIG     0x00
#define INA219_REG_SHUNT_V    0x01
#define INA219_REG_BUS_V      0x02
#define INA219_REG_CURRENT    0x04
#define INA219_REG_CALIB      0x05

// Configuration: BRNG=16V (bit13=0), PGA=/8 ±320mV (bits 12:11=11),
// BADC=9-bit (bits 10:7=0000), SADC=9-bit (bits 6:3=0000),
// MODE=shunt+bus continuous (bits 2:0=111).
#define INA219_CONFIG         ((0u<<13) | (3u<<11) | (0u<<7) | (0u<<3) | 0x07)

// CAL = trunc(0.04096 / (Current_LSB * R_shunt))
// With R_shunt = 0.01 Ω and CAL = 4096:
//   Current_LSB = 0.04096 / (4096 * 0.01) = 1 mA per LSB
// (max representable current ≈ 32 A, which is well past the shunt's PGA
//  saturation of 320 mV / 10 mΩ = 32 A — fine.)
#define INA219_CAL            4096
#define INA219_CURRENT_LSB_A  1e-3f       // 1 mA per LSB

esp_err_t ina219_init(void) {
    esp_err_t err = i2c_bus_write_word(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config write failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_bus_write_word(INA219_ADDR, INA219_REG_CALIB, INA219_CAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration write failed: %s", esp_err_to_name(err));
        return err;
    }

    // Readback verification: confirm the chip actually accepted our writes.
    // If these don't match, we're not talking to an INA219 at this address,
    // or the chip is in a weird state.
    uint16_t cfg_rb = 0, cal_rb = 0;
    i2c_bus_read_word(INA219_ADDR, INA219_REG_CONFIG, &cfg_rb);
    i2c_bus_read_word(INA219_ADDR, INA219_REG_CALIB,  &cal_rb);
    if (cfg_rb != INA219_CONFIG || cal_rb != INA219_CAL) {
        ESP_LOGW(TAG, "register readback mismatch: cfg 0x%04X (want 0x%04X), "
                      "cal 0x%04X (want 0x%04X)",
                 cfg_rb, INA219_CONFIG, cal_rb, INA219_CAL);
    }
    ESP_LOGI(TAG, "init done at 0x%02X — cfg=0x%04X cal=0x%04X "
                  "(shunt=0.01Ω, current_LSB=1mA)",
             INA219_ADDR, cfg_rb, cal_rb);
    return ESP_OK;
}

esp_err_t ina219_read(float *voltage_v, float *current_a) {
    uint16_t v_raw = 0, c_raw = 0;
    esp_err_t err = i2c_bus_read_word(INA219_ADDR, INA219_REG_BUS_V, &v_raw);
    if (err != ESP_OK) return err;
    // Bus voltage: bits [15:3] = voltage in 4 mV LSB at the IN- pin.
    // The General Driver board appears to divide the battery voltage
    // before feeding it to the chip — see CONFIG_UGV_INA219_VOLTAGE_SCALE_X1000.
    const float scale = CONFIG_UGV_INA219_VOLTAGE_SCALE_X1000 / 1000.0f;
    *voltage_v = (float)(v_raw >> 3) * 0.004f * scale;

    err = i2c_bus_read_word(INA219_ADDR, INA219_REG_CURRENT, &c_raw);
    if (err != ESP_OK) return err;
    *current_a = (float)(int16_t)c_raw * INA219_CURRENT_LSB_A;

    // Diagnostic: log the raw registers occasionally so we can compare with
    // a multimeter when something looks wrong. Throttled to ~once a minute
    // at 2 Hz battery rate.
    static uint32_t tick = 0;
    if ((tick++ % 120) == 0) {
        ESP_LOGI(TAG, "raw bus=0x%04X cur=0x%04X -> %.3fV %.3fA",
                 v_raw, c_raw, *voltage_v, *current_a);
    }
    return ESP_OK;
}
