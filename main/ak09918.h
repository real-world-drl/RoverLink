#pragma once

#include <stdbool.h>
#include "esp_err.h"

// AK09918 3-axis magnetometer (the QST companion chip on the General Driver
// board's IMU). Continuous 100 Hz mode. Sensitivity 0.15 µT/LSB.
esp_err_t ak09918_init(void);

// Reads a fresh sample if DRDY is set; otherwise leaves *fresh = false and
// returns ESP_OK without touching the output buffers. Callers should keep
// the previous values when fresh==false.
esp_err_t ak09918_read(float mag_ut[3], bool *fresh);
