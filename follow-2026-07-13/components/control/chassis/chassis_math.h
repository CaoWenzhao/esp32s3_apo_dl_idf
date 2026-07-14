#pragma once

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float i_term;
    float prev_meas;
    bool has_prev;
} chassis_pid_t;

void chassis_pid_reset(chassis_pid_t *pid);
float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt);
void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty);
