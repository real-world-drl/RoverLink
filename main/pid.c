// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "pid.h"

#include <math.h>

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(ugv_pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max, float min_drive) {
    p->kp = kp;  p->ki = ki;  p->kd = kd;
    p->out_min = out_min;  p->out_max = out_max;
    p->min_drive = min_drive;
    pid_reset(p);
}

void pid_set_tunings(ugv_pid_t *p, float kp, float ki, float kd) {
    p->kp = kp;  p->ki = ki;  p->kd = kd;
}

void pid_set_output_limits(ugv_pid_t *p, float out_min, float out_max) {
    p->out_min = out_min;  p->out_max = out_max;
    p->i_term = clampf(p->i_term, out_min, out_max);
}

void pid_set_min_drive(ugv_pid_t *p, float min_drive) {
    p->min_drive = min_drive;
}

void pid_reset(ugv_pid_t *p) {
    p->i_term = 0.0f;
    p->last_meas = 0.0f;
    p->last_us = 0;
    p->initialized = false;
}

float pid_compute(ugv_pid_t *p, float setpoint, float measurement, int64_t now_us) {
    if (!p->initialized) {
        p->last_meas = measurement;
        p->last_us = now_us;
        p->initialized = true;
        return 0.0f;
    }

    const float dt = (float)(now_us - p->last_us) * 1e-6f;
    p->last_us = now_us;
    if (dt <= 0.0f || dt > 1.0f) {
        // First sample after long pause or clock anomaly: don't punch the
        // integrator. Reuse measurement, skip output update.
        p->last_meas = measurement;
        return 0.0f;
    }

    const float error = setpoint - measurement;

    p->i_term += p->ki * error * dt;
    p->i_term  = clampf(p->i_term, p->out_min, p->out_max);

    const float dmeas = (measurement - p->last_meas) / dt;
    p->last_meas = measurement;

    float out = p->kp * error + p->i_term - p->kd * dmeas;
    out = clampf(out, p->out_min, p->out_max);

    // Minimum-drive feedforward. A real motion command whose PID output
    // lands below the motor's stiction floor would otherwise produce a PWM
    // too small to turn the wheel — that's what made turn-in-place (tiny
    // wheel setpoints) feel sluggish. Floor the magnitude to min_drive,
    // keeping the controller's sign. A commanded stop (setpoint ~0) still
    // returns a hard zero so the bot can rest and the wheels don't creep.
    if (fabsf(setpoint) < 1e-6f) {
        out = 0.0f;
    } else if (p->min_drive > 0.0f && fabsf(out) < p->min_drive) {
        out = copysignf(p->min_drive, out != 0.0f ? out : setpoint);
    }
    return out;
}
