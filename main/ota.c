// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "ota.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "sdkconfig.h"

static const char *TAG = "ota";

// URLs longer than this are rejected. A LAN http://host:port/roverlink.bin
// is well under 256 B.
#define OTA_URL_MAX  256

static volatile bool s_in_progress = false;
static volatile bool s_validated   = false;
static esp_timer_handle_t s_rollback_timer = NULL;

// ---- rollback watchdog -------------------------------------------------

#if defined(CONFIG_UGV_ENABLE_MQTT)
static void rollback_timer_cb(void *arg) {
    (void)arg;
    if (s_validated) return;
    ESP_LOGE(TAG, "image not validated within %d ms — rolling back",
             CONFIG_UGV_OTA_VALIDATE_TIMEOUT_MS);
    // Marks the running app invalid and reboots; the bootloader then reverts
    // to the previous slot. Does not return on success.
    esp_ota_mark_app_invalid_rollback_and_reboot();
}
#endif

void ota_init(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) != ESP_OK ||
        st != ESP_OTA_IMG_PENDING_VERIFY) {
        // Already confirmed valid (normal boot), or no OTA state to track.
        s_validated = true;
        return;
    }

#if defined(CONFIG_UGV_ENABLE_MQTT)
    // Defer validation until we prove the new image can reach the broker.
    ESP_LOGW(TAG, "running '%s' PENDING_VERIFY — arming %d ms rollback watchdog",
             running->label, CONFIG_UGV_OTA_VALIDATE_TIMEOUT_MS);
    const esp_timer_create_args_t args = {
        .callback = rollback_timer_cb,
        .name     = "ota_rollback",
    };
    if (esp_timer_create(&args, &s_rollback_timer) == ESP_OK) {
        esp_timer_start_once(s_rollback_timer,
            (uint64_t)CONFIG_UGV_OTA_VALIDATE_TIMEOUT_MS * 1000ULL);
    }
#else
    // No remote validator compiled in — accept the image immediately so a
    // serial-flashed, rollback-enabled build doesn't revert on next boot.
    ESP_LOGI(TAG, "no MQTT validator; marking image valid");
    ota_mark_healthy();
#endif
}

void ota_mark_healthy(void) {
    if (s_validated) return;
    s_validated = true;
    if (s_rollback_timer) {
        esp_timer_stop(s_rollback_timer);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            ESP_LOGI(TAG, "image '%s' marked valid (rollback cancelled)",
                     running->label);
        }
    }
}

// ---- update task -------------------------------------------------------

static void ota_task(void *arg) {
    char *url = (char *)arg;
    ESP_LOGW(TAG, "starting OTA from %s", url);

    esp_http_client_config_t http = {
        .url               = url,
        .timeout_ms        = 20000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t cfg = {
        .http_config = &http,
    };

    esp_err_t err = esp_https_ota(&cfg);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "OTA OK — rebooting into new image");
        free(url);
        esp_restart();   // does not return
    }
    ESP_LOGE(TAG, "OTA failed: %s — staying on current image",
             esp_err_to_name(err));
    free(url);
    s_in_progress = false;
    vTaskDelete(NULL);
}

esp_err_t ota_start(const char *url, int url_len) {
    if (url_len <= 0 || url_len >= OTA_URL_MAX) {
        ESP_LOGE(TAG, "rejecting OTA: bad url length %d", url_len);
        return ESP_ERR_INVALID_ARG;
    }
    if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "rejecting OTA: url must be http(s)://");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress — ignoring request");
        return ESP_ERR_INVALID_STATE;
    }

    char *copy = malloc(url_len + 1);
    if (!copy) return ESP_ERR_NO_MEM;
    memcpy(copy, url, url_len);
    copy[url_len] = '\0';

    s_in_progress = true;
    // 8 KB stack: TLS (for https URLs) needs the headroom; http uses less.
    if (xTaskCreate(ota_task, "ota", 8192, copy, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn OTA task");
        free(copy);
        s_in_progress = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}
