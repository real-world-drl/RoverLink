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
//     = trunc(0.04096 / (1e-4 * 0.01))   = 40960  --> clipped to int16 -> 4096
// Standard Waveshare config picks Current_LSB = 100 µA so CAL = 4096.
#define INA219_CAL            4096
#define INA219_CURRENT_LSB_A  1e-4f       // 100 µA per LSB

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
    ESP_LOGI(TAG, "ready at 0x%02X (shunt=0.01Ω, current_LSB=100µA)", INA219_ADDR);
    return ESP_OK;
}

esp_err_t ina219_read(float *voltage_v, float *current_a) {
    uint16_t raw;
    esp_err_t err = i2c_bus_read_word(INA219_ADDR, INA219_REG_BUS_V, &raw);
    if (err != ESP_OK) return err;
    // Bus voltage: bits [15:3] = voltage in 4 mV LSB.
    *voltage_v = (float)(raw >> 3) * 0.004f;

    uint16_t cur_raw;
    err = i2c_bus_read_word(INA219_ADDR, INA219_REG_CURRENT, &cur_raw);
    if (err != ESP_OK) return err;
    *current_a = (float)(int16_t)cur_raw * INA219_CURRENT_LSB_A;
    return ESP_OK;
}
