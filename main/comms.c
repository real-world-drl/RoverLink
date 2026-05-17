#include "comms.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#ifdef CONFIG_UGV_ENABLE_MQTT
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#endif

static const char *TAG = "comms";

// ---- Shared command queues (independent of any transport) --------------

static QueueHandle_t s_cmd_vel_q;
static QueueHandle_t s_cmd_pid_q;
static QueueHandle_t s_cmd_display_q;
static volatile int64_t s_cmd_display_recv_us = 0;

// Forward declaration: the UART module exports this when compiled in.
// Fallback stub keeps the preempt check well-defined when UART is off.
#ifdef CONFIG_UGV_ENABLE_UART_LINK
extern bool uart_link_recent_rx(void);
#else
static inline bool uart_link_recent_rx(void) { return false; }
#endif

void comms_queues_init(void) {
    if (s_cmd_vel_q == NULL) {
        s_cmd_vel_q     = xQueueCreate(1, sizeof(ugv_cmd_vel_t));
        s_cmd_pid_q     = xQueueCreate(1, sizeof(ugv_cmd_pid_t));
        s_cmd_display_q = xQueueCreate(1, sizeof(ugv_cmd_display_t));
    }
}

void comms_push_cmd_vel(const ugv_cmd_vel_t *v, bool from_uart) {
    if (!from_uart && uart_link_recent_rx()) {
        // UART is currently the live control channel; drop the MQTT push
        // so a stale broker packet can't stomp on a fresh wire packet.
        return;
    }
    xQueueOverwrite(s_cmd_vel_q, v);
}

void comms_push_cmd_pid(const ugv_cmd_pid_t *v) {
    xQueueOverwrite(s_cmd_pid_q, v);
}

void comms_push_cmd_display(const ugv_cmd_display_t *v) {
    xQueueOverwrite(s_cmd_display_q, v);
    s_cmd_display_recv_us = esp_timer_get_time();
}

bool comms_take_cmd_vel    (ugv_cmd_vel_t     *out) { return xQueueReceive(s_cmd_vel_q,     out, 0) == pdTRUE; }
bool comms_take_cmd_pid    (ugv_cmd_pid_t     *out) { return xQueueReceive(s_cmd_pid_q,     out, 0) == pdTRUE; }
bool comms_take_cmd_display(ugv_cmd_display_t *out) { return xQueueReceive(s_cmd_display_q, out, 0) == pdTRUE; }

int64_t comms_cmd_display_recv_us(void) { return s_cmd_display_recv_us; }

// ---- MQTT transport ----------------------------------------------------

#ifdef CONFIG_UGV_ENABLE_MQTT

#define TOPIC_BUF_LEN 96

static esp_mqtt_client_handle_t s_mqtt;
static volatile bool s_mqtt_connected = false;

static char s_topic_cmd_vel[TOPIC_BUF_LEN];
static char s_topic_cmd_pid[TOPIC_BUF_LEN];
static char s_topic_cmd_display[TOPIC_BUF_LEN];
static char s_topic_tel_wheel[TOPIC_BUF_LEN];
static char s_topic_tel_imu[TOPIC_BUF_LEN];
static char s_topic_tel_batt[TOPIC_BUF_LEN];
static char s_topic_status[TOPIC_BUF_LEN];

static EventGroupHandle_t s_wifi_events;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_wifi_retries = 0;

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_mqtt_connected = false;
        if (s_wifi_retries < CONFIG_UGV_WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_wifi_retries++;
            ESP_LOGW(TAG, "wifi retry %d/%d",
                     s_wifi_retries, CONFIG_UGV_WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retries = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void) {
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_evt, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_evt, NULL, NULL));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_UGV_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_UGV_WIFI_PASS,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

static void handle_incoming(const char *topic, int topic_len,
                            const char *data,  int data_len) {
    if (topic_len == (int)strlen(s_topic_cmd_vel) &&
        memcmp(topic, s_topic_cmd_vel, topic_len) == 0) {
        if (data_len != sizeof(ugv_cmd_vel_t)) {
            ESP_LOGW(TAG, "cmd_vel wrong size: %d (want %d)",
                     data_len, (int)sizeof(ugv_cmd_vel_t));
            return;
        }
        ugv_cmd_vel_t v;
        memcpy(&v, data, sizeof(v));
        comms_push_cmd_vel(&v, /*from_uart=*/false);
    } else if (topic_len == (int)strlen(s_topic_cmd_pid) &&
               memcmp(topic, s_topic_cmd_pid, topic_len) == 0) {
        if (data_len != sizeof(ugv_cmd_pid_t)) {
            ESP_LOGW(TAG, "cmd_pid wrong size: %d (want %d)",
                     data_len, (int)sizeof(ugv_cmd_pid_t));
            return;
        }
        ugv_cmd_pid_t v;
        memcpy(&v, data, sizeof(v));
        comms_push_cmd_pid(&v);
    } else if (topic_len == (int)strlen(s_topic_cmd_display) &&
               memcmp(topic, s_topic_cmd_display, topic_len) == 0) {
        if (data_len != sizeof(ugv_cmd_display_t)) {
            ESP_LOGW(TAG, "cmd_display wrong size: %d (want %d)",
                     data_len, (int)sizeof(ugv_cmd_display_t));
            return;
        }
        ugv_cmd_display_t v;
        memcpy(&v, data, sizeof(v));
        comms_push_cmd_display(&v);
    }
}

static void mqtt_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt connected");
        s_mqtt_connected = true;
        esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd_vel, 1);
        esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd_pid, 1);
        esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd_display, 1);
        esp_mqtt_client_publish(s_mqtt, s_topic_status,
                                UGV_STATUS_ONLINE, 0, 1, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "mqtt disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_DATA:
        handle_incoming(e->topic, e->topic_len, e->data, e->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "mqtt error");
        break;
    default:
        break;
    }
}

static void build_topics(void) {
#define BUILD(buf, suffix) \
    snprintf((buf), TOPIC_BUF_LEN, "ugv/%s/%s/%s", \
             CONFIG_UGV_ROBOT_ID, UGV_TOPIC_VERSION, (suffix))
    BUILD(s_topic_cmd_vel,     UGV_TOPIC_CMD_VEL);
    BUILD(s_topic_cmd_pid,     UGV_TOPIC_CMD_PID);
    BUILD(s_topic_cmd_display, UGV_TOPIC_CMD_DISPLAY);
    BUILD(s_topic_tel_wheel,   UGV_TOPIC_TEL_WHEEL);
    BUILD(s_topic_tel_imu,     UGV_TOPIC_TEL_IMU);
    BUILD(s_topic_tel_batt,    UGV_TOPIC_TEL_BATT);
    BUILD(s_topic_status,      UGV_TOPIC_STATUS);
#undef BUILD
    ESP_LOGI(TAG, "topics:");
    ESP_LOGI(TAG, "  sub: %s", s_topic_cmd_vel);
    ESP_LOGI(TAG, "  sub: %s", s_topic_cmd_pid);
    ESP_LOGI(TAG, "  sub: %s", s_topic_cmd_display);
    ESP_LOGI(TAG, "  pub: %s", s_topic_tel_wheel);
    ESP_LOGI(TAG, "  pub: %s", s_topic_tel_imu);
    ESP_LOGI(TAG, "  pub: %s", s_topic_tel_batt);
    ESP_LOGI(TAG, "  lwt: %s", s_topic_status);
}

static esp_err_t mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_UGV_MQTT_BROKER_URI,
        .credentials = {
            .username = (strlen(CONFIG_UGV_MQTT_USERNAME) > 0)
                            ? CONFIG_UGV_MQTT_USERNAME : NULL,
            .authentication.password = (strlen(CONFIG_UGV_MQTT_PASSWORD) > 0)
                            ? CONFIG_UGV_MQTT_PASSWORD : NULL,
        },
        .session = {
            .last_will = {
                .topic   = s_topic_status,
                .msg     = UGV_STATUS_OFFLINE,
                .msg_len = (int)strlen(UGV_STATUS_OFFLINE),
                .qos     = 1,
                .retain  = 1,
            },
        },
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    if (!s_mqtt) return ESP_FAIL;
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, &mqtt_evt, NULL);
    return esp_mqtt_client_start(s_mqtt);
}

esp_err_t comms_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    build_topics();

    if (wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "wifi failed to associate; continuing with MQTT disabled");
        return ESP_FAIL;
    }
    return mqtt_init();
}

void comms_publish_wheel(const ugv_wheel_telem_t *t) {
    if (!s_mqtt_connected) return;
    esp_mqtt_client_publish(s_mqtt, s_topic_tel_wheel,
                            (const char *)t, sizeof(*t), 0, 0);
}

void comms_publish_battery(const ugv_battery_telem_t *t) {
    if (!s_mqtt_connected) return;
    esp_mqtt_client_publish(s_mqtt, s_topic_tel_batt,
                            (const char *)t, sizeof(*t), 0, 0);
}

void comms_publish_imu(const ugv_imu_telem_t *t) {
    if (!s_mqtt_connected) return;
    esp_mqtt_client_publish(s_mqtt, s_topic_tel_imu,
                            (const char *)t, sizeof(*t), 0, 0);
}

bool comms_mqtt_connected(void) { return s_mqtt_connected; }

#else  // CONFIG_UGV_ENABLE_MQTT not defined ----------------------------

esp_err_t comms_init(void) { return ESP_OK; }
void comms_publish_wheel  (const ugv_wheel_telem_t   *t) { (void)t; }
void comms_publish_battery(const ugv_battery_telem_t *t) { (void)t; }
void comms_publish_imu    (const ugv_imu_telem_t     *t) { (void)t; }
bool comms_mqtt_connected (void) { return false; }

#endif
