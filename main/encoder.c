#include "encoder.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "encoder";

// Stock Waveshare General Driver pin assignment.
// GPIO 34 and 35 are input-only on ESP32; fine for encoder inputs, but they
// have no internal pull-up/pull-down circuitry. The IDF pulse_cnt driver
// unconditionally calls gpio_pullup_en() on the edge/level GPIOs and logs a
// "GPIO number error" for these two pins at init — cosmetic, no flag to
// suppress it. Encoder still works provided the board has external pull-ups
// on the AENCA/AENCB lines (the Waveshare board does).
#define PIN_AENCA  35
#define PIN_AENCB  34
#define PIN_BENCA  27
#define PIN_BENCB  16

// Internal HW counter is 16-bit on ESP32. With accum_count enabled, the
// driver auto-accumulates whenever a watchpoint at +HI / -LO is hit, so we
// get effective 32-bit counts.
#define PCNT_HIGH_LIMIT   10000
#define PCNT_LOW_LIMIT   -10000

// Kconfig bools emit no macro at all when unset, so promote them to 0/1.
#ifdef CONFIG_UGV_ENCODER_LEFT_INVERT
#  define ENC_LEFT_INVERT 1
#else
#  define ENC_LEFT_INVERT 0
#endif
#ifdef CONFIG_UGV_ENCODER_RIGHT_INVERT
#  define ENC_RIGHT_INVERT 1
#else
#  define ENC_RIGHT_INVERT 0
#endif

static pcnt_unit_handle_t s_unit_left  = NULL;
static pcnt_unit_handle_t s_unit_right = NULL;

static esp_err_t setup_unit(int edge_gpio, int level_gpio,
                            pcnt_unit_handle_t *out_unit) {
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, out_unit));

    pcnt_glitch_filter_config_t glitch_cfg = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*out_unit, &glitch_cfg));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = edge_gpio,
        .level_gpio_num = level_gpio,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &chan_cfg, &chan));

    // 2x decode: count both edges of the edge channel, use level channel
    // for direction. Matches Arduino ESP32Encoder::attachHalfQuad().
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // on positive edge
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); // on negative edge
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // when level is high
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); // when level is low

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*out_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*out_unit, PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(*out_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*out_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(*out_unit));
    return ESP_OK;
}

esp_err_t encoder_init(void) {
    ESP_ERROR_CHECK(setup_unit(PIN_AENCA, PIN_AENCB, &s_unit_left));
    ESP_ERROR_CHECK(setup_unit(PIN_BENCA, PIN_BENCB, &s_unit_right));
    ESP_LOGI(TAG, "PCNT encoders ready (left A/B=%d/%d, right A/B=%d/%d)",
             PIN_AENCA, PIN_AENCB, PIN_BENCA, PIN_BENCB);
    return ESP_OK;
}

int32_t encoder_left_ticks(void) {
    int c = 0;
    pcnt_unit_get_count(s_unit_left, &c);
    return ENC_LEFT_INVERT ? -(int32_t)c : (int32_t)c;
}

int32_t encoder_right_ticks(void) {
    int c = 0;
    pcnt_unit_get_count(s_unit_right, &c);
    return ENC_RIGHT_INVERT ? -(int32_t)c : (int32_t)c;
}
