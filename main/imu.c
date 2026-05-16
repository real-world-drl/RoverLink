#include "imu.h"
#include "ak09918.h"
#include "qmi8658.h"

#include "esp_log.h"

static const char *TAG = "imu";

static float s_last_mag[3] = {0.0f, 0.0f, 0.0f};

esp_err_t imu_init(void) {
    esp_err_t err = qmi8658_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmi8658 init failed");
        return err;
    }
    err = ak09918_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ak09918 init failed — continuing without magnetometer");
        // Don't propagate: a missing mag chip shouldn't kill IMU publishing.
    }
    return ESP_OK;
}

esp_err_t imu_read(float acc_ms2[3], float gyr_rads[3], float mag_ut[3],
                   float *temp_c, bool *mag_fresh) {
    esp_err_t err = qmi8658_read(acc_ms2, gyr_rads, temp_c);
    if (err != ESP_OK) return err;

    bool fresh = false;
    float mag_new[3];
    if (ak09918_read(mag_new, &fresh) == ESP_OK && fresh) {
        s_last_mag[0] = mag_new[0];
        s_last_mag[1] = mag_new[1];
        s_last_mag[2] = mag_new[2];
    }
    mag_ut[0] = s_last_mag[0];
    mag_ut[1] = s_last_mag[1];
    mag_ut[2] = s_last_mag[2];
    if (mag_fresh) *mag_fresh = fresh;
    return ESP_OK;
}
