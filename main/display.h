#pragma once

#include "esp_err.h"

// Initializes the OLED panel. Safe to call once at boot.
esp_err_t display_init(void);

// FreeRTOS task entry. Runs at CONFIG_UGV_DISPLAY_HZ.
// Prefers values from the most recent cmd/display packet (host pose) if
// received within CONFIG_UGV_DISPLAY_HOST_TIMEOUT_MS; otherwise falls back
// to the firmware odometry estimate. A single-letter indicator (H/L) in
// the top-right shows which source is currently driving the display.
void display_task(void *arg);
