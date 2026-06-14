// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include "esp_err.h"

// Validate-on-boot + rollback watchdog. Call once early in app_main.
// If the running image is PENDING_VERIFY (i.e. it just arrived via OTA, or
// is the first boot after a serial flash with rollback enabled):
//   - with MQTT compiled in, arms a timer that rolls the image back unless
//     ota_mark_healthy() is called within CONFIG_UGV_OTA_VALIDATE_TIMEOUT_MS;
//   - without MQTT (no remote validator), marks the image valid immediately.
// A no-op when the image is already confirmed valid.
void ota_init(void);

// Marks the running image valid and cancels the rollback watchdog. Call
// once the firmware has proven itself healthy — currently on MQTT connect,
// which proves the new image can still reach the host to receive a fix.
void ota_mark_healthy(void);

// Kicks off a background firmware download from `url` ("http://..." on the
// LAN, or "https://..."). Returns immediately; on success the device
// reboots into the new image, on failure it stays on the current one.
// `url` need not be NUL-terminated — `url_len` bytes are copied.
esp_err_t ota_start(const char *url, int url_len);
