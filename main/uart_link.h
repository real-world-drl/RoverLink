// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "ugv_packets.h"

// Sets up UART2 (default GPIO 19 TX / 18 RX, 921600 8N1) and spawns the
// RX framing task. Idempotent — calling twice is a no-op.
esp_err_t uart_link_init(void);

// TX side: serialize the packet into [sync][type][len][payload][crc8] and
// emit it in a single uart_write_bytes() call. Safe to call from any task
// (the IDF driver locks per call; we build the frame in a stack buffer so
// concurrent calls never interleave byte-wise).
void uart_link_publish_wheel  (const ugv_wheel_telem_t   *t);
void uart_link_publish_imu    (const ugv_imu_telem_t     *t);
void uart_link_publish_battery(const ugv_battery_telem_t *t);
void uart_link_publish_status (const char *online_or_offline);

// True if a valid inbound frame was decoded within
// CONFIG_UGV_UART_PREEMPT_MS. Used by comms_push_cmd_vel() to drop MQTT
// commands while the UART link is actively driving the bot.
bool uart_link_recent_rx(void);
