// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Peter Bohm

#pragma once

#include <stdint.h>

typedef struct {
    float wheel_diameter_m;
    float track_width_m;
    int   encoder_ppr;          // pulses per wheel revolution after gearing
    float meters_per_tick;      // pi * D / PPR, pre-computed
    float max_wheel_mps;        // achievable wheel speed ceiling; 0 = no limit
} kinematics_t;

void  kinematics_init(kinematics_t *k,
                      float wheel_diameter_m,
                      float track_width_m,
                      int encoder_ppr,
                      float max_wheel_mps);

// Differential-drive inverse kinematics: cmd_vel -> per-wheel linear velocity.
// v_left, v_right are in m/s of wheel circumference (i.e. ground speed of
// that wheel, ignoring slip).
//
// When max_wheel_mps > 0, applies angular-priority saturation: if a wheel
// would exceed the ceiling, the common (linear) component is reduced to keep
// the differential (angular) intact, so the robot keeps turning at speed
// rather than saturating both wheels and driving straight.
void  kinematics_cmd_vel_to_wheels(const kinematics_t *k,
                                   float linear_x, float angular_z,
                                   float *v_left_mps, float *v_right_mps);

// Convenience for telemetry: convert tick delta to meters.
float kinematics_ticks_to_meters(const kinematics_t *k, int32_t ticks);
