#include "ak09918.h"
#include "i2c_bus.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ak09918";

#define AK_ADDR         0x0C
#define REG_WIA1        0x00
#define REG_WIA2        0x01
#define REG_ST1         0x10
#define REG_HXL         0x11    // 6 data bytes then ST2 (terminator)
#define REG_CNTL2       0x31
#define REG_CNTL3       0x32

#define MODE_CONT_100HZ 0x08
#define SRST_BIT        0x01
#define DRDY_BIT        0x01

#define MAG_UT_PER_LSB  0.15f

esp_err_t ak09918_init(void) {
    uint8_t wia1 = 0, wia2 = 0;
    i2c_bus_read_reg(AK_ADDR, REG_WIA1, &wia1);
    i2c_bus_read_reg(AK_ADDR, REG_WIA2, &wia2);
    if (wia1 != 0x48 || wia2 != 0x0C) {
        ESP_LOGW(TAG, "unexpected WIA 0x%02X/0x%02X (expected 0x48/0x0C) — continuing",
                 wia1, wia2);
    }

    // Soft reset, then continuous 100 Hz.
    i2c_bus_write_reg(AK_ADDR, REG_CNTL3, SRST_BIT);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t err = i2c_bus_write_reg(AK_ADDR, REG_CNTL2, MODE_CONT_100HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mode write failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "ready at 0x%02X (continuous 100 Hz)", AK_ADDR);
    return ESP_OK;
}

esp_err_t ak09918_read(float mag_ut[3], bool *fresh) {
    *fresh = false;
    uint8_t st1 = 0;
    esp_err_t err = i2c_bus_read_reg(AK_ADDR, REG_ST1, &st1);
    if (err != ESP_OK) return err;
    if (!(st1 & DRDY_BIT)) return ESP_OK;

    // Reading HXL..ST2 (7 bytes) atomically. Reading through ST2 is required
    // to release the DRDY latch — without it the next sample never appears.
    uint8_t buf[7];
    err = i2c_bus_read_regs(AK_ADDR, REG_HXL, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    const int16_t hx = (int16_t)(buf[0] | (buf[1] << 8));
    const int16_t hy = (int16_t)(buf[2] | (buf[3] << 8));
    const int16_t hz = (int16_t)(buf[4] | (buf[5] << 8));

    mag_ut[0] = (float)hx * MAG_UT_PER_LSB;
    mag_ut[1] = (float)hy * MAG_UT_PER_LSB;
    mag_ut[2] = (float)hz * MAG_UT_PER_LSB;
    *fresh = true;
    return ESP_OK;
}
