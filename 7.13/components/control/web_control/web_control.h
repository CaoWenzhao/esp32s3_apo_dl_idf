#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    WEB_CONTROL_MODE_AUTO = 0,
    WEB_CONTROL_MODE_MANUAL,
} web_control_mode_t;

typedef struct {
    const char *ap_ssid;
    const char *ap_password;
    uint32_t command_timeout_ms;
    float max_manual_linear_mps;
    float max_manual_angular_rps;
    uint8_t default_follow_speed_pct;
    uint8_t default_follow_turn_pct;
    uint8_t default_remote_speed_pct;
} web_control_config_t;

typedef struct {
    web_control_mode_t mode;
    bool estop_latched;
    bool client_alive;
    float manual_linear_mps;
    float manual_angular_rps;
    uint8_t follow_speed_pct;
    uint8_t follow_turn_pct;
    uint8_t remote_speed_pct;
    uint32_t command_age_ms;
} web_control_command_t;

typedef struct {
    const char *state;
    bool uwb_ok;
    bool lidar_ok;
    bool ultrasonic_left_ok;
    bool ultrasonic_right_ok;
    bool fsr_ok;
    bool encoder_ok;
    float target_distance_m;
    float target_bearing_rad;
    float front_clearance_m;
    float fsr_voltage_v;
    float fsr_weight_kg;
    int fsr_raw;
    float measured_linear_mps;
    float measured_angular_rps;
    int left_pulse_us;
    int right_pulse_us;
} web_control_telemetry_t;

web_control_config_t web_control_default_config(void);
esp_err_t web_control_init(const web_control_config_t *config);
void web_control_get_command(web_control_command_t *command);
void web_control_publish(const web_control_telemetry_t *telemetry);
