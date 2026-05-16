#pragma once

#include <stdint.h>
#include "esp_err.h"

// Initializes the two H-bridge motor channels using LEDC PWM and direction
// pins. Pins are fixed for the Waveshare General Driver board; see motor.c.
esp_err_t motor_init(void);

// Set signed PWM, range [-255, 255]. Positive = forward (modulo per-side
// invert flags from Kconfig). Out-of-range values are clamped. Safe to call
// from any task context.
void motor_set_left_pwm(int16_t pwm);
void motor_set_right_pwm(int16_t pwm);

// Cuts PWM and floats the H-bridge inputs. Idempotent.
void motor_stop_all(void);
