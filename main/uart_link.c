// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "uart_link.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "comms.h"

static const char *TAG = "uart_link";

#define UART_PORT       ((uart_port_t)CONFIG_UGV_UART_PORT)
#define UART_RX_BUF     1024
#define UART_TX_BUF     0           // 0 = blocking writes; we send one frame at a time

// Largest payload is ugv_imu_telem_t = 56 B; full frame max = 1+1+1+56+1 = 60 B.
#define UART_MAX_PAYLOAD  64
#define UART_MAX_FRAME    (3 + UART_MAX_PAYLOAD + 1)

static volatile int64_t s_last_good_rx_us = 0;
static bool s_started = false;

// Per-source counters, logged every ~5 seconds from the RX task.
typedef struct {
    uint32_t rx_ok;
    uint32_t rx_bad_crc;
    uint32_t rx_bad_len;
    uint32_t rx_bad_type;
    uint32_t rx_resync;
} uart_stats_t;
static uart_stats_t s_stats;

// ---- CRC8 (poly 0x07, init 0x00) — bitwise, cheap enough at our rates --

static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ---- TX side -----------------------------------------------------------

static void send_frame(ugv_pkt_type_t type, const void *payload, size_t len) {
    if (!s_started || len > UART_MAX_PAYLOAD) return;
    uint8_t buf[UART_MAX_FRAME];
    buf[0] = UGV_UART_SYNC;
    buf[1] = (uint8_t)type;
    buf[2] = (uint8_t)len;
    if (len > 0) memcpy(&buf[3], payload, len);
    buf[3 + len] = crc8(&buf[1], 2 + len);   // covers type+len+payload
    uart_write_bytes(UART_PORT, (const char *)buf, 3 + len + 1);
}

void uart_link_publish_wheel(const ugv_wheel_telem_t *t) {
    send_frame(UGV_PKT_TEL_WHEEL, t, sizeof(*t));
}
void uart_link_publish_imu(const ugv_imu_telem_t *t) {
    send_frame(UGV_PKT_TEL_IMU, t, sizeof(*t));
}
void uart_link_publish_battery(const ugv_battery_telem_t *t) {
    send_frame(UGV_PKT_TEL_BATT, t, sizeof(*t));
}
void uart_link_publish_status(const char *s) {
    send_frame(UGV_PKT_STATUS, s, strlen(s));
}

// ---- RX side -----------------------------------------------------------

static int expected_payload_len(uint8_t type) {
    switch (type) {
    case UGV_PKT_CMD_VEL:     return sizeof(ugv_cmd_vel_t);
    case UGV_PKT_CMD_PID:     return sizeof(ugv_cmd_pid_t);
    case UGV_PKT_CMD_DISPLAY: return sizeof(ugv_cmd_display_t);
    default: return -1;        // includes telemetry/status types — bot
                               // doesn't accept those as input.
    }
}

static void dispatch(uint8_t type, const uint8_t *payload) {
    switch (type) {
    case UGV_PKT_CMD_VEL: {
        ugv_cmd_vel_t v;
        memcpy(&v, payload, sizeof(v));
        comms_push_cmd_vel(&v, /*from_uart=*/true);
        break;
    }
    case UGV_PKT_CMD_PID: {
        ugv_cmd_pid_t v;
        memcpy(&v, payload, sizeof(v));
        comms_push_cmd_pid(&v);
        break;
    }
    case UGV_PKT_CMD_DISPLAY: {
        ugv_cmd_display_t v;
        memcpy(&v, payload, sizeof(v));
        comms_push_cmd_display(&v);
        break;
    }
    default: break;  // already filtered by expected_payload_len
    }
}

typedef enum {
    ST_WAIT_SYNC,
    ST_READ_TYPE,
    ST_READ_LEN,
    ST_READ_PAYLOAD,
    ST_READ_CRC,
} rx_state_t;

static void uart_rx_task(void *arg) {
    (void)arg;
    rx_state_t state = ST_WAIT_SYNC;
    uint8_t  cur_type = 0;
    int      cur_len  = 0;
    int      payload_idx = 0;
    uint8_t  payload[UART_MAX_PAYLOAD];

    int64_t  last_log_us = esp_timer_get_time();

    for (;;) {
        uint8_t b;
        int n = uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(100));
        if (n != 1) {
            // No byte this window — periodic counter dump and continue.
            int64_t now = esp_timer_get_time();
            if (now - last_log_us > 5 * 1000 * 1000LL) {
                ESP_LOGI(TAG, "rx ok=%u bad_crc=%u bad_len=%u bad_type=%u resync=%u",
                         (unsigned)s_stats.rx_ok, (unsigned)s_stats.rx_bad_crc,
                         (unsigned)s_stats.rx_bad_len, (unsigned)s_stats.rx_bad_type,
                         (unsigned)s_stats.rx_resync);
                last_log_us = now;
            }
            continue;
        }

        switch (state) {
        case ST_WAIT_SYNC:
            if (b == UGV_UART_SYNC) state = ST_READ_TYPE;
            break;

        case ST_READ_TYPE: {
            int expected = expected_payload_len(b);
            if (expected < 0) {
                s_stats.rx_bad_type++;
                s_stats.rx_resync++;
                state = ST_WAIT_SYNC;
                break;
            }
            cur_type = b;
            cur_len  = expected;
            state = ST_READ_LEN;
            break;
        }

        case ST_READ_LEN:
            if ((int)b != cur_len) {
                s_stats.rx_bad_len++;
                s_stats.rx_resync++;
                state = ST_WAIT_SYNC;
                break;
            }
            payload_idx = 0;
            state = (cur_len > 0) ? ST_READ_PAYLOAD : ST_READ_CRC;
            break;

        case ST_READ_PAYLOAD:
            payload[payload_idx++] = b;
            if (payload_idx >= cur_len) state = ST_READ_CRC;
            break;

        case ST_READ_CRC: {
            // CRC is over [type, len, payload...] — rebuild that contiguous
            // span on the stack so we can hash it in one call.
            uint8_t hdr_pl[2 + UART_MAX_PAYLOAD];
            hdr_pl[0] = cur_type;
            hdr_pl[1] = (uint8_t)cur_len;
            if (cur_len > 0) memcpy(&hdr_pl[2], payload, cur_len);
            uint8_t expect = crc8(hdr_pl, 2 + cur_len);
            if (expect == b) {
                dispatch(cur_type, payload);
                s_stats.rx_ok++;
                s_last_good_rx_us = esp_timer_get_time();
            } else {
                s_stats.rx_bad_crc++;
                s_stats.rx_resync++;
            }
            state = ST_WAIT_SYNC;
            break;
        }
        }
    }
}

// ---- Public API --------------------------------------------------------

bool uart_link_recent_rx(void) {
    if (s_last_good_rx_us == 0) return false;
    int64_t age = esp_timer_get_time() - s_last_good_rx_us;
    return age < (int64_t)CONFIG_UGV_UART_PREEMPT_MS * 1000LL;
}

esp_err_t uart_link_init(void) {
    if (s_started) return ESP_OK;

    uart_config_t cfg = {
        .baud_rate  = CONFIG_UGV_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT,
        CONFIG_UGV_UART_TX_PIN, CONFIG_UGV_UART_RX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT,
        UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 3072, NULL, 7, NULL, 0);
    s_started = true;

    ESP_LOGI(TAG, "UART%d ready: TX=%d RX=%d @ %d baud (preempt window %d ms)",
             CONFIG_UGV_UART_PORT, CONFIG_UGV_UART_TX_PIN, CONFIG_UGV_UART_RX_PIN,
             CONFIG_UGV_UART_BAUD, CONFIG_UGV_UART_PREEMPT_MS);
    uart_link_publish_status(UGV_STATUS_ONLINE);
    return ESP_OK;
}
