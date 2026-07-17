#include "chassis_math.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void chassis_pid_reset(chassis_pid_t *pid)
{
    if (pid == NULL) return;
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->has_prev = false;
}

float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt)
{
    if (pid == NULL) return 0.0f;
    if (dt <= 0.0f) dt = 1e-3f;
    const float error = setpoint - measured;
    pid->i_term = clampf(pid->i_term + pid->ki * error * dt,
                         pid->out_min, pid->out_max);
    float derivative = 0.0f;
    if (pid->has_prev) derivative = (measured - pid->prev_meas) / dt;
    pid->prev_meas = measured;
    pid->has_prev = true;
    return clampf(pid->kp * error + pid->i_term - pid->kd * derivative,
                  pid->out_min, pid->out_max);
}

void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty)
{
    float left = 0.0f;
    float right = 0.0f;
    if (max_speed_mps > 1e-6f) {
        const float half_track = 0.5f * track_width_m;
        left = (v_mps - omega_rps * half_track) / max_speed_mps;
        right = (v_mps + omega_rps * half_track) / max_speed_mps;
    }
    float magnitude = fmaxf(fabsf(left), fabsf(right));
    if (magnitude > 1.0f) {
        left /= magnitude;
        right /= magnitude;
    }
    if (left_duty != NULL) *left_duty = clampf(left, -1.0f, 1.0f);
    if (right_duty != NULL) *right_duty = clampf(right, -1.0f, 1.0f);
}
