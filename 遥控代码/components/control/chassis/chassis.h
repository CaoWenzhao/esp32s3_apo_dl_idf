#pragma once

/*
 * chassis.h - CLOSED-LOOP differential-drive chassis for the follow-me suitcase.
 *             (算法2: real APO-DL ESC + RC PWM + AB encoder speed loop)
 *
 * Mechanics (from the project hardware):
 *   - Two REAR wheels are driven motors; the two FRONT wheels are passive
 *     casters (universal wheels). Classic rear-axle differential drive: a body
 *     command (v, omega) is realised by spinning the two rear wheels at
 *     different speeds; the front casters follow freely.
 *
 * Drive electronics (this is what makes 算法2 different from 算法1):
 *   - Each motor is driven by an **APO-DL ESC** that takes an **RC servo pulse**
 *     (50 Hz, 1000..2000 us, 1500 us = neutral/stop). >1500 us = forward,
 *     <1500 us = reverse. The pulse is generated with LEDC (14-bit @ 50 Hz),
 *     exactly like the standalone 动力轮代码 branch.
 *   - Each motor has an **AB quadrature encoder**. We decode it 4x in a GPIO
 *     ISR (same lookup table as 动力轮代码) and run a **per-wheel PID speed
 *     loop**, so chassis_set_velocity() now commands real wheel speeds in m/s
 *     instead of an open-loop duty guess. A feed-forward term maps the desired
 *     speed straight to a pulse so the robot still moves even if an encoder is
 *     disconnected (graceful degradation), and the PID trims out the error.
 *
 * Usage from the control loop (fixed rate):
 *     chassis_set_velocity(&ch, v, omega);   // set the (v, omega) setpoint
 *     chassis_update(&ch, dt_s);             // read encoders, run PID, drive ESC
 * Call chassis_update() every cycle; it also runs a fail-safe that ramps the
 * ESC back to neutral if no fresh setpoint arrives within failsafe_timeout_s.
 *
 * Coordinate / sign convention (shared with follow_avoid):
 *   v > 0 = forward, omega > 0 = turn LEFT (CCW).
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* ===================================================================== PID */
/*
 * Minimal PID for one wheel's speed loop. Pure C (no hardware), so it is
 * unit-tested on the PC alongside the kinematics. Output units are ESC
 * microseconds (offset added on top of the feed-forward); derivative is taken
 * on the measurement to avoid set-point kicks, and the integrator is clamped
 * against the same output limits (anti-windup).
 */
typedef struct {
    float kp;          /* us per (m/s) */
    float ki;          /* us per (m/s . s) */
    float kd;          /* us per (m/s / s) */
    float out_min;     /* clamp on the PID contribution (us) */
    float out_max;
    /* state */
    float i_term;
    float prev_meas;
    bool has_prev;
} chassis_pid_t;

void chassis_pid_reset(chassis_pid_t *pid);
float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt);

/* ============================================================== encoder */
typedef struct {
    int a_gpio;
    int b_gpio;
    int8_t sign;               /* +1 normal, -1 if wired/mounted reversed */
    volatile int64_t count;    /* 4x quadrature tick count */
    volatile uint8_t last_state;
} chassis_enc_t;

/* ============================================================== config */
typedef struct {
    /* ---- ESC RC-PWM output (LEDC) ---- */
    int esc_freq_hz;           /* 50 for standard RC ESC */
    int esc_period_us;         /* 20000 (= 1/50 Hz) */
    int esc_min_us;            /* 1000 = full reverse */
    int esc_mid_us;            /* 1500 = neutral / stop */
    int esc_max_us;            /* 2000 = full forward */
    int pwm_resolution_bits;   /* 14 -> plenty for 50 Hz pulse timing */
    int ledc_speed_mode;       /* LEDC_LOW_SPEED_MODE on ESP32-S3 */
    int ledc_timer;            /* LEDC_TIMER_0.. */
    int left_ledc_channel;     /* LEDC_CHANNEL_0.. */
    int right_ledc_channel;

    int left_esc_gpio;         /* rear-left ESC signal pin  (动力轮代码: GPIO4) */
    int right_esc_gpio;        /* rear-right ESC signal pin (动力轮代码: GPIO5) */
    bool left_invert;          /* flip forward/reverse for this wheel */
    bool right_invert;

    /* ---- AB encoders ---- */
    int left_enc_a_gpio;       /* 动力轮代码: GPIO6 */
    int left_enc_b_gpio;       /* 动力轮代码: GPIO7 */
    int right_enc_a_gpio;      /* 动力轮代码: GPIO15 */
    int right_enc_b_gpio;      /* 动力轮代码: GPIO16 */
    bool left_enc_invert;      /* flip the sign of the left count */
    bool right_enc_invert;
    float ticks_per_meter;     /* CALIBRATE: 4x ticks per metre of travel */

    /* ---- kinematics / limits ---- */
    float track_width_m;       /* distance between the two rear wheels (m) */
    float max_speed_mps;       /* top wheel speed; sets the feed-forward gain
                                  and clamps the speed setpoints */

    /* ---- speed PID (same gains for both wheels) ---- */
    float kp;
    float ki;
    float kd;
    float pid_out_limit_us;    /* clamp on each wheel's PID contribution (us) */

    /* ---- safety ---- */
    float slew_us_per_s;       /* max ESC pulse change rate (0 disables) */
    float failsafe_timeout_s;  /* no fresh setpoint this long -> neutral */
} chassis_config_t;

/* ============================================================== state */
typedef struct {
    chassis_config_t cfg;
    bool initialized;
    int duty_max;              /* (1 << resolution) - 1 */
    float ff_us_per_mps;       /* (esc_max-esc_mid)/max_speed, derived once */

    chassis_enc_t enc_l;
    chassis_enc_t enc_r;
    int64_t last_count_l;
    int64_t last_count_r;
    float meas_left_mps;       /* latest encoder-measured wheel speeds */
    float meas_right_mps;

    chassis_pid_t pid_l;
    chassis_pid_t pid_r;
    float target_left_mps;     /* wheel-speed set-points (from set_velocity) */
    float target_right_mps;

    float cmd_pulse_l_us;      /* last applied ESC pulse (for slew) */
    float cmd_pulse_r_us;
    float since_setpoint_s;    /* fail-safe watchdog */

    /* dead-reckoning odometry (from encoders) */
    double odo_x_m;
    double odo_y_m;
    double odo_yaw_rad;
} chassis_t;

/* ============================================================== API */

/* Sensible defaults matching the 动力轮代码 wiring; override via Kconfig. */
chassis_config_t chassis_default_config(void);

esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg);
void chassis_deinit(chassis_t *ch);

/*
 * Set the body-velocity set-point. v_mps + = forward, omega_rps + = turn LEFT.
 * Internally mixed into per-wheel speed targets; the PID is run by
 * chassis_update(). Cheap to call; does not touch hardware by itself.
 */
esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps);

/* Set per-wheel speed targets directly (m/s), bypassing the mixer. */
esp_err_t chassis_set_wheel_speeds(chassis_t *ch, float left_mps,
                                   float right_mps);

/* Low-level manual override: write raw ESC pulses (us). Use for bring-up only;
 * does NOT run the PID. Pulses are clamped to [esc_min_us, esc_max_us]. */
esp_err_t chassis_set_pulse_us(chassis_t *ch, int left_us, int right_us);

/*
 * Run one closed-loop step: snapshot the encoders, compute wheel speeds, run the
 * per-wheel PID toward the current set-point, and write the ESC pulses. Must be
 * called periodically (e.g. at the control-loop rate). dt_s <= 0 is treated as a
 * small default.
 */
esp_err_t chassis_update(chassis_t *ch, float dt_s);

/* Latest encoder feedback (any out-pointer may be NULL). */
void chassis_get_measured(chassis_t *ch, float *v_mps, float *omega_rps,
                          float *left_mps, float *right_mps);

/* Dead-reckoning pose accumulated from the encoders (any pointer may be NULL). */
void chassis_get_odometry(chassis_t *ch, float *x_m, float *y_m, float *yaw_rad);

/* Coast to neutral (ESC 1500 us) and reset the speed set-points + integrators. */
void chassis_stop(chassis_t *ch);

/* Same as stop for an RC ESC (no separate electrical brake line). Kept for API
 * compatibility; also clears the PID integrators. */
void chassis_brake(chassis_t *ch);

/* ---- Pure kinematics (no hardware; unit-testable on PC) ------------------ */

/*
 * Differential-drive mixing into normalised wheel "speeds" in [-1, 1]:
 *   left  = (v - omega*track/2) / max_speed
 *   right = (v + omega*track/2) / max_speed
 * If either would exceed +-1 both are scaled together so the turn geometry is
 * preserved instead of clipped. (算法2 multiplies the result by max_speed to get
 * the m/s set-points, but the normalised form keeps the tests unchanged.)
 */
void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty);
