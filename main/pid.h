#pragma once

#include <stdbool.h>
#include <stdint.h>

// Note: name is `ugv_pid_t` (not `pid_t`) to avoid clashing with POSIX
// pid_t from <sys/types.h>, which gets pulled in transitively by IDF.
typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float deadband;

    float i_term;
    float last_meas;
    int64_t last_us;
    bool initialized;
} ugv_pid_t;

void  pid_init(ugv_pid_t *p,
               float kp, float ki, float kd,
               float out_min, float out_max,
               float deadband);

void  pid_set_tunings(ugv_pid_t *p, float kp, float ki, float kd);
void  pid_set_output_limits(ugv_pid_t *p, float out_min, float out_max);
void  pid_set_deadband(ugv_pid_t *p, float deadband);

void  pid_reset(ugv_pid_t *p);

// Returns output clamped to [out_min, out_max]. Derivative is on the
// measurement (not the error) to avoid kick on setpoint changes.
// Integral is clamped to keep total output inside the saturation envelope.
float pid_compute(ugv_pid_t *p, float setpoint, float measurement, int64_t now_us);
