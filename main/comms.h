#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "ugv_packets.h"

// Create the shared command queues. Call once from app_main before any
// transport (MQTT, UART) starts. Idempotent — safe to call when no
// transports are enabled (queues just go unused).
void comms_queues_init(void);

// Brings up NVS, WiFi (STA), event loop, netif, then starts the MQTT client.
// Blocks until WiFi associates (or retry budget is exhausted). MQTT reconnects
// internally; callers should not depend on broker availability at return.
esp_err_t comms_init(void);

// --- Producer-side: push a freshly-received command into the shared queue.
// Any transport (MQTT, UART, ...) calls these. For cmd_vel only, `from_uart`
// gates the preempt policy: an MQTT push is dropped while UART has been
// active within CONFIG_UGV_UART_PREEMPT_MS so a stale MQTT packet can't
// stomp on a live UART control stream.
void comms_push_cmd_vel    (const ugv_cmd_vel_t     *v, bool from_uart);
void comms_push_cmd_pid    (const ugv_cmd_pid_t     *v);
void comms_push_cmd_display(const ugv_cmd_display_t *v);

// Non-blocking pop of the *latest* received command. Returns true if a fresh
// value was waiting since the last call. (Queue length 1, overwriting — older
// commands are silently dropped, which is the desired behavior for cmd_vel.)
bool comms_take_cmd_vel(ugv_cmd_vel_t *out);
bool comms_take_cmd_pid(ugv_cmd_pid_t *out);
bool comms_take_cmd_display(ugv_cmd_display_t *out);

// Device-clock timestamp of the most recent cmd/display reception (0 if
// none yet this session). The display task uses this for staleness checks.
int64_t comms_cmd_display_recv_us(void);

// MQTT publishes (QoS 0, no retain). Drops silently if MQTT isn't connected
// or isn't compiled in.
void comms_publish_wheel(const ugv_wheel_telem_t *t);
void comms_publish_battery(const ugv_battery_telem_t *t);
void comms_publish_imu(const ugv_imu_telem_t *t);

// True once the MQTT client has reported MQTT_EVENT_CONNECTED at least once
// in the current session.
bool comms_mqtt_connected(void);
