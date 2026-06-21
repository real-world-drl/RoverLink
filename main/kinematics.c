// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "kinematics.h"

#include <math.h>

void kinematics_init(kinematics_t *k,
                     float wheel_diameter_m, float track_width_m,
                     int encoder_ppr, float max_wheel_mps) {
    k->wheel_diameter_m = wheel_diameter_m;
    k->track_width_m    = track_width_m;
    k->encoder_ppr      = encoder_ppr;
    k->meters_per_tick  = (float)M_PI * wheel_diameter_m / (float)encoder_ppr;
    k->max_wheel_mps    = max_wheel_mps;
}

void kinematics_cmd_vel_to_wheels(const kinematics_t *k,
                                  float linear_x, float angular_z,
                                  float *v_left_mps, float *v_right_mps) {
    const float half_w = 0.5f * k->track_width_m;
    float vl = linear_x - angular_z * half_w;
    float vr = linear_x + angular_z * half_w;

    // Angular-priority saturation. Split into common (linear) and
    // differential (turn) parts; if the peak wheel exceeds the achievable
    // ceiling, shrink the common part first so the differential — and hence
    // the turn rate — survives. Without this, commanding linear at/above the
    // ceiling pins both wheels at max and the bot can't turn while moving.
    const float vmax = k->max_wheel_mps;
    if (vmax > 0.0f) {
        const float common = 0.5f * (vl + vr);   // == linear_x
        float       diff   = 0.5f * (vr - vl);   // == angular_z * half_w
        // A turn that alone exceeds the ceiling can't be fully honoured —
        // cap it (you can't spin faster than the wheels turn).
        if (fabsf(diff) > vmax) diff = copysignf(vmax, diff);
        const float room = vmax - fabsf(diff);   // >= 0: headroom left for linear
        float c = common;
        if (fabsf(c) > room) c = copysignf(room, c);
        vl = c - diff;
        vr = c + diff;
    }

    *v_left_mps  = vl;
    *v_right_mps = vr;
}

float kinematics_ticks_to_meters(const kinematics_t *k, int32_t ticks) {
    return (float)ticks * k->meters_per_tick;
}
