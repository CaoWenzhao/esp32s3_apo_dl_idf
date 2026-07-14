#include "chassis.h"

#include <math.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

/* ===================================================================== *
 * Pure section (compiled on BOTH the ESP target and the PC test build).  *
 * No hardware here: kinematics + PID only.                              *
 * ===================================================================== */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void chassis_pid_reset(chassis_pid_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->has_prev = false;
}

float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt)
{
    if (pid == NULL) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        dt = 1e-3f;
    }
    const float err = setpoint - measured;

    /* Integrate with anti-windup: the integrator alone is clamped to the same
     * limits as the output, so it can never push the command past saturation. */
    pid->i_term += pid->ki * err * dt;
    pid->i_term = clampf(pid->i_term, pid->out_min, pid->out_max);

    /* Derivative on measurement (not on error) avoids a spike when the
     * set-point steps. */
    float deriv = 0.0f;
    if (pid->has_prev) {
        deriv = (measured - pid->prev_meas) / dt;
    }
    pid->prev_meas = measured;
    pid->has_prev = true;

    float out = pid->kp * err + pid->i_term - pid->kd * deriv;
    return clampf(out, pid->out_min, pid->out_max);
}

void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty)
{
    float l = 0.0f;
    float r = 0.0f;
    if (max_speed_mps > 1e-6f) {
        const float half = 0.5f * track_width_m;
        l = (v_mps - omega_rps * half) / max_speed_mps;
        r = (v_mps + omega_rps * half) / max_speed_mps;
    }

    /* If saturated, scale both wheels equally to preserve the turn ratio. */
    float m = fabsf(l);
    if (fabsf(r) > m) {
        m = fabsf(r);
    }
    if (m > 1.0f) {
        l /= m;
        r /= m;
    }

    if (left_duty) {
        *left_duty = clampf(l, -1.0f, 1.0f);
    }
    if (right_duty) {
        *right_duty = clampf(r, -1.0f, 1.0f);
    }
}

chassis_config_t chassis_default_config(void)
{
    chassis_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* ESC RC-PWM (matches the 动力轮代码 branch). */
    cfg.esc_freq_hz = 50;
    cfg.esc_period_us = 20000;
    cfg.esc_min_us = 1000;
    cfg.esc_mid_us = 1500;
    cfg.esc_max_us = 2000;
    cfg.pwm_resolution_bits = 14;
    cfg.ledc_speed_mode = 0; /* LEDC_LOW_SPEED_MODE */
    cfg.ledc_timer = 0;      /* LEDC_TIMER_0 */
    cfg.left_ledc_channel = 0;
    cfg.right_ledc_channel = 1;
    cfg.left_esc_gpio = 4;
    cfg.right_esc_gpio = 5;
    cfg.left_invert = false;
    cfg.right_invert = false;

    /* AB encoders (matches the 动力轮代码 branch). */
    cfg.left_enc_a_gpio = 6;
    cfg.left_enc_b_gpio = 7;
    cfg.right_enc_a_gpio = 15;
    cfg.right_enc_b_gpio = 16;
    cfg.left_enc_invert = false;
    cfg.right_enc_invert = false;
    cfg.ticks_per_meter = 2000.0f; /* CALIBRATE for your wheel + encoder */

    cfg.track_width_m = 0.30f;
    cfg.max_speed_mps = 0.8f;

    cfg.kp = 200.0f;  /* us per (m/s) */
    cfg.ki = 300.0f;  /* us per (m/s . s) */
    cfg.kd = 5.0f;
    cfg.pid_out_limit_us = 400.0f;

    cfg.slew_us_per_s = 1500.0f;  /* ESC ramps ~1 full range per 0.33 s */
    cfg.failsafe_timeout_s = 0.3f;
    return cfg;
}

/* ===================================================================== *
 * ESP target implementation.                                            *
 * ===================================================================== */
static const char *TAG = "chassis";

/* Counts shared with the encoder ISR are guarded by this spinlock. */
static portMUX_TYPE s_enc_mux = portMUX_INITIALIZER_UNLOCKED;

/* 4x quadrature decode table, indexed by (prev_state << 2) | new_state. */
static const int8_t quad_table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
};

static void IRAM_ATTR enc_isr(void *arg)
{
    chassis_enc_t *e = (chassis_enc_t *)arg;
    const int a = gpio_get_level(e->a_gpio);
    const int b = gpio_get_level(e->b_gpio);
    const uint8_t now = (uint8_t)((a << 1) | b);
    const uint8_t idx = (uint8_t)((e->last_state << 2) | now);

    portENTER_CRITICAL_ISR(&s_enc_mux);
    e->count += (int64_t)e->sign * quad_table[idx];
    e->last_state = now;
    portEXIT_CRITICAL_ISR(&s_enc_mux);
}

static uint32_t pulse_us_to_duty(const chassis_t *ch, float pulse_us)
{
    const chassis_config_t *cfg = &ch->cfg;
    float p = clampf(pulse_us, (float)cfg->esc_min_us, (float)cfg->esc_max_us);
    float duty = p * (float)ch->duty_max / (float)cfg->esc_period_us;
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > (float)ch->duty_max) {
        duty = (float)ch->duty_max;
    }
    return (uint32_t)(duty + 0.5f);
}

static void write_pulse(chassis_t *ch, int channel, float pulse_us)
{
    const chassis_config_t *cfg = &ch->cfg;
    const uint32_t duty = pulse_us_to_duty(ch, pulse_us);
    ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode, (ledc_channel_t)channel,
                  duty);
    ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                     (ledc_channel_t)channel);
}

static float slew(float target, float current, float max_step)
{
    if (max_step <= 0.0f) {
        return target;
    }
    float d = target - current;
    if (d > max_step) {
        d = max_step;
    } else if (d < -max_step) {
        d = -max_step;
    }
    return current + d;
}

static esp_err_t cfg_esc_channel(const chassis_config_t *cfg, int channel,
                                 int gpio)
{
    ledc_channel_config_t ch = {
        .gpio_num = gpio,
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,
        .channel = (ledc_channel_t)channel,
        .timer_sel = (ledc_timer_t)cfg->ledc_timer,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    return ledc_channel_config(&ch);
}

static esp_err_t setup_encoder(chassis_enc_t *e, int a_gpio, int b_gpio,
                               bool invert)
{
    e->a_gpio = a_gpio;
    e->b_gpio = b_gpio;
    e->sign = invert ? -1 : 1;
    e->count = 0;

    if (a_gpio < 0 || b_gpio < 0) {
        return ESP_OK; /* encoder optional: feed-forward still drives the ESC */
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << a_gpio) | (1ULL << b_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        return ret;
    }
    e->last_state =
        (uint8_t)((gpio_get_level(a_gpio) << 1) | gpio_get_level(b_gpio));

    /* 4x decoding needs an edge interrupt on BOTH channels. */
    ret = gpio_isr_handler_add(a_gpio, enc_isr, e);
    if (ret != ESP_OK) {
        return ret;
    }
    return gpio_isr_handler_add(b_gpio, enc_isr, e);
}

esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg)
{
    if (ch == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(ch, 0, sizeof(*ch));
    ch->cfg = *cfg;
    ch->duty_max = (1 << cfg->pwm_resolution_bits) - 1;

    const float range = (float)(cfg->esc_max_us - cfg->esc_mid_us);
    ch->ff_us_per_mps =
        (cfg->max_speed_mps > 1e-6f) ? (range / cfg->max_speed_mps) : 0.0f;

    /* ---- ESC PWM ---- */
    ledc_timer_config_t tcfg = {
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,
        .timer_num = (ledc_timer_t)cfg->ledc_timer,
        .duty_resolution = (ledc_timer_bit_t)cfg->pwm_resolution_bits,
        .freq_hz = cfg->esc_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&tcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", ret);
        return ret;
    }
    ret = cfg_esc_channel(cfg, cfg->left_ledc_channel, cfg->left_esc_gpio);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = cfg_esc_channel(cfg, cfg->right_ledc_channel, cfg->right_esc_gpio);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Hold neutral so the ESC can arm. */
    ch->cmd_pulse_l_us = (float)cfg->esc_mid_us;
    ch->cmd_pulse_r_us = (float)cfg->esc_mid_us;
    write_pulse(ch, cfg->left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, cfg->right_ledc_channel, ch->cmd_pulse_r_us);

    /* ---- Encoders ---- */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", isr_ret);
        return isr_ret;
    }
    ret = setup_encoder(&ch->enc_l, cfg->left_enc_a_gpio, cfg->left_enc_b_gpio,
                        cfg->left_enc_invert);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "left encoder setup failed: %d", ret);
        return ret;
    }
    ret = setup_encoder(&ch->enc_r, cfg->right_enc_a_gpio,
                        cfg->right_enc_b_gpio, cfg->right_enc_invert);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "right encoder setup failed: %d", ret);
        return ret;
    }

    /* ---- PID ---- */
    ch->pid_l.kp = ch->pid_r.kp = cfg->kp;
    ch->pid_l.ki = ch->pid_r.ki = cfg->ki;
    ch->pid_l.kd = ch->pid_r.kd = cfg->kd;
    ch->pid_l.out_min = ch->pid_r.out_min = -cfg->pid_out_limit_us;
    ch->pid_l.out_max = ch->pid_r.out_max = cfg->pid_out_limit_us;
    chassis_pid_reset(&ch->pid_l);
    chassis_pid_reset(&ch->pid_r);

    ch->initialized = true;
    ESP_LOGI(TAG,
             "init: ESC L=GPIO%d R=GPIO%d @%dHz | enc L=%d/%d R=%d/%d | "
             "track=%.2fm vmax=%.2fm/s ff=%.0fus/(m/s)",
             cfg->left_esc_gpio, cfg->right_esc_gpio, cfg->esc_freq_hz,
             cfg->left_enc_a_gpio, cfg->left_enc_b_gpio, cfg->right_enc_a_gpio,
             cfg->right_enc_b_gpio, cfg->track_width_m, cfg->max_speed_mps,
             ch->ff_us_per_mps);
    return ESP_OK;
}

esp_err_t chassis_set_wheel_speeds(chassis_t *ch, float left_mps,
                                   float right_mps)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const float vmax = ch->cfg.max_speed_mps;
    ch->target_left_mps = clampf(left_mps, -vmax, vmax);
    ch->target_right_mps = clampf(right_mps, -vmax, vmax);
    ch->since_setpoint_s = 0.0f; /* a fresh command resets the fail-safe */
    return ESP_OK;
}

esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    float l = 0.0f;
    float r = 0.0f;
    chassis_diff_drive_mix(v_mps, omega_rps, ch->cfg.track_width_m,
                           ch->cfg.max_speed_mps, &l, &r);
    return chassis_set_wheel_speeds(ch, l * ch->cfg.max_speed_mps,
                                    r * ch->cfg.max_speed_mps);
}

esp_err_t chassis_set_pulse_us(chassis_t *ch, int left_us, int right_us)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ch->cmd_pulse_l_us =
        clampf((float)left_us, (float)ch->cfg.esc_min_us,
               (float)ch->cfg.esc_max_us);
    ch->cmd_pulse_r_us =
        clampf((float)right_us, (float)ch->cfg.esc_min_us,
               (float)ch->cfg.esc_max_us);
    write_pulse(ch, ch->cfg.left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, ch->cfg.right_ledc_channel, ch->cmd_pulse_r_us);
    return ESP_OK;
}

esp_err_t chassis_update(chassis_t *ch, float dt_s)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const chassis_config_t *cfg = &ch->cfg;
    if (dt_s <= 0.0f) {
        dt_s = 0.02f;
    }

    /* Fail-safe: if the control loop stops feeding set-points, coast to stop. */
    ch->since_setpoint_s += dt_s;
    if (ch->since_setpoint_s > cfg->failsafe_timeout_s) {
        ch->target_left_mps = 0.0f;
        ch->target_right_mps = 0.0f;
    }

    /* Snapshot the ISR-updated counts. */
    int64_t cl;
    int64_t cr;
    portENTER_CRITICAL(&s_enc_mux);
    cl = ch->enc_l.count;
    cr = ch->enc_r.count;
    portEXIT_CRITICAL(&s_enc_mux);

    const int64_t dl = cl - ch->last_count_l;
    const int64_t dr = cr - ch->last_count_r;
    ch->last_count_l = cl;
    ch->last_count_r = cr;

    const float tpm = (cfg->ticks_per_meter > 1.0f) ? cfg->ticks_per_meter : 1.0f;
    const float left_dist = (float)dl / tpm;
    const float right_dist = (float)dr / tpm;
    ch->meas_left_mps = left_dist / dt_s;
    ch->meas_right_mps = right_dist / dt_s;

    /* Dead-reckoning odometry. */
    const double dc = 0.5 * ((double)left_dist + (double)right_dist);
    const double track = (cfg->track_width_m > 1e-3f) ? cfg->track_width_m : 1e-3;
    ch->odo_yaw_rad += ((double)right_dist - (double)left_dist) / track;
    ch->odo_x_m += dc * cos(ch->odo_yaw_rad);
    ch->odo_y_m += dc * sin(ch->odo_yaw_rad);

    /* Speed loop: feed-forward (open-loop guess) + PID trim on encoder error. */
    const float vmax = cfg->max_speed_mps;
    const float tl = clampf(ch->target_left_mps, -vmax, vmax);
    const float tr = clampf(ch->target_right_mps, -vmax, vmax);

    float off_l = ch->ff_us_per_mps * tl +
                  chassis_pid_step(&ch->pid_l, tl, ch->meas_left_mps, dt_s);
    float off_r = ch->ff_us_per_mps * tr +
                  chassis_pid_step(&ch->pid_r, tr, ch->meas_right_mps, dt_s);

    /* A zero target means "stop": force neutral and drop the integrator so the
     * ESC does not creep. */
    if (fabsf(tl) < 1e-4f) {
        off_l = 0.0f;
        chassis_pid_reset(&ch->pid_l);
    }
    if (fabsf(tr) < 1e-4f) {
        off_r = 0.0f;
        chassis_pid_reset(&ch->pid_r);
    }

    float pulse_l = (float)cfg->esc_mid_us + (cfg->left_invert ? -off_l : off_l);
    float pulse_r =
        (float)cfg->esc_mid_us + (cfg->right_invert ? -off_r : off_r);

    /* Slew-limit the pulse for current-spike / gearbox protection. */
    const float max_step = cfg->slew_us_per_s * dt_s;
    pulse_l = slew(pulse_l, ch->cmd_pulse_l_us, max_step);
    pulse_r = slew(pulse_r, ch->cmd_pulse_r_us, max_step);
    ch->cmd_pulse_l_us = pulse_l;
    ch->cmd_pulse_r_us = pulse_r;

    write_pulse(ch, cfg->left_ledc_channel, pulse_l);
    write_pulse(ch, cfg->right_ledc_channel, pulse_r);
    return ESP_OK;
}

void chassis_get_measured(chassis_t *ch, float *v_mps, float *omega_rps,
                          float *left_mps, float *right_mps)
{
    if (ch == NULL) {
        return;
    }
    const float l = ch->meas_left_mps;
    const float r = ch->meas_right_mps;
    if (v_mps) {
        *v_mps = 0.5f * (l + r);
    }
    if (omega_rps) {
        const float track =
            (ch->cfg.track_width_m > 1e-3f) ? ch->cfg.track_width_m : 1e-3f;
        *omega_rps = (r - l) / track;
    }
    if (left_mps) {
        *left_mps = l;
    }
    if (right_mps) {
        *right_mps = r;
    }
}

void chassis_get_odometry(chassis_t *ch, float *x_m, float *y_m, float *yaw_rad)
{
    if (ch == NULL) {
        return;
    }
    if (x_m) {
        *x_m = (float)ch->odo_x_m;
    }
    if (y_m) {
        *y_m = (float)ch->odo_y_m;
    }
    if (yaw_rad) {
        *yaw_rad = (float)ch->odo_yaw_rad;
    }
}

void chassis_stop(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    ch->target_left_mps = 0.0f;
    ch->target_right_mps = 0.0f;
    ch->since_setpoint_s = 0.0f;
    chassis_pid_reset(&ch->pid_l);
    chassis_pid_reset(&ch->pid_r);
    ch->cmd_pulse_l_us = (float)ch->cfg.esc_mid_us;
    ch->cmd_pulse_r_us = (float)ch->cfg.esc_mid_us;
    write_pulse(ch, ch->cfg.left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, ch->cfg.right_ledc_channel, ch->cmd_pulse_r_us);
}

void chassis_brake(chassis_t *ch)
{
    /* An RC ESC has no separate brake line; commanding neutral is the strongest
     * stop available without driving reverse. */
    chassis_stop(ch);
}

void chassis_deinit(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    chassis_stop(ch);
    if (ch->enc_l.a_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_l.a_gpio);
    }
    if (ch->enc_l.b_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_l.b_gpio);
    }
    if (ch->enc_r.a_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_r.a_gpio);
    }
    if (ch->enc_r.b_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_r.b_gpio);
    }
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.left_ledc_channel, 0);
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.right_ledc_channel, 0);
    ch->initialized = false;
}

