#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "ugv_packets.h"

// Brings up NVS, WiFi (STA), event loop, netif, then starts the MQTT client.
// Blocks until WiFi associates (or retry budget is exhausted). MQTT reconnects
// internally; callers should not depend on broker availability at return.
esp_err_t comms_init(void);

// Non-blocking pop of the *latest* received command. Returns true if a fresh
// value was waiting since the last call. (Queue length 1, overwriting — older
// commands are silently dropped, which is the desired behavior for cmd_vel.)
bool comms_take_cmd_vel(ugv_cmd_vel_t *out);
bool comms_take_cmd_pid(ugv_cmd_pid_t *out);
bool comms_take_cmd_display(ugv_cmd_display_t *out);

// Device-clock timestamp of the most recent cmd/display reception (0 if
// none yet this session). The display task uses this for staleness checks.
int64_t comms_cmd_display_recv_us(void);

// Publishes (QoS 0, no retain). Drops silently if MQTT isn't connected yet.
void comms_publish_wheel(const ugv_wheel_telem_t *t);
void comms_publish_battery(const ugv_battery_telem_t *t);
void comms_publish_imu(const ugv_imu_telem_t *t);

// True once the MQTT client has reported MQTT_EVENT_CONNECTED at least once
// in the current session.
bool comms_mqtt_connected(void);
