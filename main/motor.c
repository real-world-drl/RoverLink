#include "motor.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "motor";

// Pin assignments for the Waveshare General Driver (ESP32 WROOM-32).
// Matches the stock Arduino firmware's ugv_config.h.
#define PIN_PWMA   25
#define PIN_AIN1   21
#define PIN_AIN2   17
#define PIN_PWMB   26
#define PIN_BIN1   22
#define PIN_BIN2   23

#define PWM_FREQ_HZ      100000
#define PWM_RESOLUTION   LEDC_TIMER_8_BIT
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_CHAN_LEFT    LEDC_CHANNEL_0
#define PWM_CHAN_RIGHT   LEDC_CHANNEL_1
#define PWM_MAX          ((1 << 8) - 1)

// Kconfig bools emit no macro at all when unset, so promote them to 0/1.
#ifdef CONFIG_UGV_MOTOR_LEFT_INVERT
#  define MOTOR_LEFT_INVERT true
#else
#  define MOTOR_LEFT_INVERT false
#endif
#ifdef CONFIG_UGV_MOTOR_RIGHT_INVERT
#  define MOTOR_RIGHT_INVERT true
#else
#  define MOTOR_RIGHT_INVERT false
#endif

static inline int16_t clamp_pwm(int32_t v) {
    if (v >  PWM_MAX) return  PWM_MAX;
    if (v < -PWM_MAX) return -PWM_MAX;
    return (int16_t)v;
}

static void set_channel(ledc_channel_t chan, int ain1, int ain2,
                        bool invert, int16_t pwm) {
    if (invert) pwm = -pwm;

    if (pwm == 0) {
        gpio_set_level(ain1, 0);
        gpio_set_level(ain2, 0);
        ledc_set_duty(PWM_MODE, chan, 0);
        ledc_update_duty(PWM_MODE, chan);
        return;
    }

    if (pwm > 0) {
        gpio_set_level(ain1, 0);
        gpio_set_level(ain2, 1);
    } else {
        gpio_set_level(ain1, 1);
        gpio_set_level(ain2, 0);
        pwm = -pwm;
    }
    ledc_set_duty(PWM_MODE, chan, (uint32_t)pwm);
    ledc_update_duty(PWM_MODE, chan);
}

esp_err_t motor_init(void) {
    const gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2)
                      | (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));
    gpio_set_level(PIN_AIN1, 0); gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0); gpio_set_level(PIN_BIN2, 0);

    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t left_cfg = {
        .gpio_num   = PIN_PWMA,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHAN_LEFT,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    const ledc_channel_config_t right_cfg = {
        .gpio_num   = PIN_PWMB,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHAN_RIGHT,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_cfg));
    ESP_ERROR_CHECK(ledc_channel_config(&right_cfg));

    ESP_LOGI(TAG, "motor PWM @ %d Hz, %d-bit, max=%d",
             PWM_FREQ_HZ, 8, PWM_MAX);
    return ESP_OK;
}

void motor_set_left_pwm(int16_t pwm) {
    set_channel(PWM_CHAN_LEFT, PIN_AIN1, PIN_AIN2,
                MOTOR_LEFT_INVERT, clamp_pwm(pwm));
}

void motor_set_right_pwm(int16_t pwm) {
    set_channel(PWM_CHAN_RIGHT, PIN_BIN1, PIN_BIN2,
                MOTOR_RIGHT_INVERT, clamp_pwm(pwm));
}

void motor_stop_all(void) {
    gpio_set_level(PIN_AIN1, 0); gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0); gpio_set_level(PIN_BIN2, 0);
    ledc_set_duty(PWM_MODE, PWM_CHAN_LEFT,  0);
    ledc_set_duty(PWM_MODE, PWM_CHAN_RIGHT, 0);
    ledc_update_duty(PWM_MODE, PWM_CHAN_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHAN_RIGHT);
}
