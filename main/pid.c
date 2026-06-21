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

    const float dmeas = (measurement - p->last_meas) / dt;
    p->last_meas = measurement;

    // Commanded stop: hard zero AND drop the integrator. If we kept
    // integrating here, the still-moving wheel (error = -measurement, large
    // ki) would wind i_term hard in the *opposite* sign during the
    // deceleration; that stale term then kicked the wheels the wrong way at
    // the start of the next move (drive forward → stop → turn lurched
    // backward before spinning). Resetting on stop is what prevents it.
    if (fabsf(setpoint) < 1e-6f) {
        p->i_term = 0.0f;
        return 0.0f;
    }

    const float error = setpoint - measurement;

    // Tentative integral, then conditional anti-windup: once the output is
    // saturated, don't let the integrator keep growing further into the
    // rail (it would only have to unwind later, causing overshoot — acute
    // with this firmware's large ki).
    float i_term = p->i_term + p->ki * error * dt;
    i_term = clampf(i_term, p->out_min, p->out_max);

    float out = p->kp * error + i_term - p->kd * dmeas;
    if (out > p->out_max) {
        out = p->out_max;
        if (error < 0.0f) p->i_term = i_term;   // commit only if it unwinds
    } else if (out < p->out_min) {
        out = p->out_min;
        if (error > 0.0f) p->i_term = i_term;
    } else {
        p->i_term = i_term;
    }

    // Stiction break, gated on the wheel being stalled. A weak PID output
    // can't move a *stopped* wheel (turn-in-place: tiny setpoints), so floor
    // it to min_drive to break free. But only while the wheel is lagging far
    // below its command — once it's rolling, kinetic friction is lower and
    // it tracks fine. Crucially, flooring a *moving* wheel would clamp both
    // wheels of a gentle arc up to the same min_drive, erasing the
    // wheel-to-wheel differential so the bot drives straight instead of
    // turning. Gating on "stalled" keeps the floor for the stuck case and
    // leaves moving turns to the PID. STALL_FRAC of the commanded speed is
    // the "still basically stuck" line.
    const float STALL_FRAC = 0.5f;
    const bool stalled = fabsf(measurement) < STALL_FRAC * fabsf(setpoint);
    if (p->min_drive > 0.0f && stalled && fabsf(out) < p->min_drive) {
        out = copysignf(p->min_drive, out != 0.0f ? out : setpoint);
    }
    return out;
}
