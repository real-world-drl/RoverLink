// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#include "kinematics.h"

#include <math.h>

void kinematics_init(kinematics_t *k,
                     float wheel_diameter_m, float track_width_m,
                     int encoder_ppr) {
    k->wheel_diameter_m = wheel_diameter_m;
    k->track_width_m    = track_width_m;
    k->encoder_ppr      = encoder_ppr;
    k->meters_per_tick  = (float)M_PI * wheel_diameter_m / (float)encoder_ppr;
}

void kinematics_cmd_vel_to_wheels(const kinematics_t *k,
                                  float linear_x, float angular_z,
                                  float *v_left_mps, float *v_right_mps) {
    const float half_w = 0.5f * k->track_width_m;
    *v_left_mps  = linear_x - angular_z * half_w;
    *v_right_mps = linear_x + angular_z * half_w;
}

float kinematics_ticks_to_meters(const kinematics_t *k, int32_t ticks) {
    return (float)ticks * k->meters_per_tick;
}
