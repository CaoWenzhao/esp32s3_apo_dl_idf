/* UWB follow vehicle with lidar avoidance and web/manual control. */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "a02yyuw.h"
#include "board_pin_config.h"
#include "ble_control.h"
#include "bu_uwb.h"
#include "chassis.h"
#include "follow_avoid.h"
#include "fsr_adc.h"
#include "rplidar_c1.h"
#include "web_control.h"

static const char *TAG = "algorithm6";

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD(value) ((value) * PI_F / 180.0f)
#define LIDAR_SECTORS 48
#define LIDAR_FOV_RAD DEG_TO_RAD(130.0f)
#define LIDAR_FRONT_MAX_DEG 65.0f
#define LIDAR_FRONT_MIN_DEG 295.0f
#define TARGET_FRESH_US 2000000ULL
#define FIELD_FRESH_US 500000ULL
#define ULTRA_FRESH_US 800000ULL
#define FSR_FRESH_US 500000ULL
#define BATTERY_FRESH_US 500000ULL
#define UWB_SMOOTH_TAU_S 0.08f
#define UWB_MAX_TARGET_SPEED_MPS 12.0f
#define UWB_JUMP_MARGIN_M 1.00f
#define UWB_REINIT_OUTLIERS 1
#define SUITCASE_BASE_WEIGHT_KG 10.2f
#define PRESSURE_DIVIDER_KOHM 8.67f
#define PRESSURE_EXCITATION_V 3.3f
#define PRESSURE_NO_LOAD_KOHM 500.0f
#define PRESSURE_CONDUCTANCE_SLOPE 0.0019f
#define PRESSURE_CONDUCTANCE_OFFSET (-0.0008f)
#define PRESSURE_MAX_KG 100.0f
#define LIDAR_TELEMETRY_MAX_M 12.0f
#define LIDAR_MOTOR_RPM 600

typedef struct {
    SemaphoreHandle_t lock;
    float target_distance_m;
    float target_bearing_rad;
    float target_bearing_rate_rps;
    uint64_t target_timestamp_us;
    fa_obstacle_field_t field;
    uint64_t field_timestamp_us;
    float ultrasonic_left_m;
    uint64_t ultrasonic_left_timestamp_us;
    uint32_t ultrasonic_left_frames;
    float ultrasonic_right_m;
    uint64_t ultrasonic_right_timestamp_us;
    uint32_t ultrasonic_right_frames;
    float fsr_voltage_v;
    float fsr_weight_kg;
    int fsr_raw;
    uint64_t fsr_timestamp_us;
    float battery_voltage_v;
    float battery_percent;
    float battery_adc_voltage_v;
    int battery_raw;
    uint64_t battery_timestamp_us;
} sensor_snapshot_t;

typedef struct {
    a02yyuw_t *device;
    bool left;
} ultrasonic_task_arg_t;

static sensor_snapshot_t s_sensors;
static chassis_t s_chassis;
static rplidar_c1_t s_lidar;
static a02yyuw_t s_ultrasonic_left;
static a02yyuw_t s_ultrasonic_right;
static ultrasonic_task_arg_t s_ultrasonic_left_arg = {
    .device = &s_ultrasonic_left,
    .left = true,
};
static ultrasonic_task_arg_t s_ultrasonic_right_arg = {
    .device = &s_ultrasonic_right,
    .left = false,
};

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void sensors_lock(void)
{
    xSemaphoreTake(s_sensors.lock, portMAX_DELAY);
}

static void sensors_unlock(void)
{
    xSemaphoreGive(s_sensors.lock);
}

static bool is_fresh(uint64_t current_us, uint64_t timestamp_us,
                     uint64_t maximum_age_us)
{
    return timestamp_us != 0 && current_us >= timestamp_us &&
           current_us - timestamp_us < maximum_age_us;
}

static float clampf_local(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static bool imu_i2c_scan_bus(int sda_gpio, int scl_gpio, const char *label)
{
    const gpio_config_t pin_config = {
        .pin_bit_mask = (1ULL << sda_gpio) | (1ULL << scl_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pin_config);
    vTaskDelay(pdMS_TO_TICKS(2));
    const int sda_level = gpio_get_level(sda_gpio);
    const int scl_level = gpio_get_level(scl_gpio);
    ESP_LOGI("imu_scan", "%s SDA=GPIO%d level=%d, SCL=GPIO%d level=%d",
             label, sda_gpio, sda_level, scl_gpio, scl_level);
    if (!sda_level || !scl_level) {
        ESP_LOGW("imu_scan", "%s bus held low; check power, GND and wiring",
                 label);
        return false;
    }

    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t ret = i2c_new_master_bus(&config, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE("imu_scan", "%s I2C init failed: %s", label,
                 esp_err_to_name(ret));
        return false;
    }

    unsigned found = 0;
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        if (i2c_master_probe(bus, address, 20) == ESP_OK) {
            ESP_LOGI("imu_scan", "%s device found at 0x%02X", label,
                     address);
            found++;
        }
    }
    i2c_del_master_bus(bus);
    return found > 0;
}

static void imu_i2c_scan(void)
{
    if (imu_i2c_scan_bus(PIN_IMU_SDA, PIN_IMU_SCL, "reserved wiring")) {
        return;
    }
    if (!imu_i2c_scan_bus(PIN_IMU_SCL, PIN_IMU_SDA, "SDA/SCL swapped")) {
        ESP_LOGW("imu_scan", "no communicating IMU found on GPIO41/42");
    }
}

static void uwb_task(void *argument)
{
    (void)argument;
    char line[BU_UWB_LINE_MAX];
    bool filter_initialized = false;
    float filtered_forward_m = 0.0f;
    float filtered_left_m = 0.0f;
    float filtered_bearing_rad = 0.0f;
    uint64_t accepted_timestamp_us = 0;
    unsigned consecutive_outliers = 0;
    while (true) {
        esp_err_t ret = bu_uwb_read_line(line, sizeof(line), 500);
        if (ret != ESP_OK) {
            continue;
        }
        bu_uwb_twr_reading_t twr = {0};
        if (!bu_uwb_parse_twr_line(line, &twr) || !twr.valid) {
            continue;
        }

        const uint64_t timestamp_us = now_us();
        const float forward_m = (float)twr.y_cm / 100.0f;
        const float left_m = (float)twr.x_cm / 100.0f;
        float dt = 0.0f;
        if (accepted_timestamp_us != 0 && timestamp_us > accepted_timestamp_us) {
            dt = (float)(timestamp_us - accepted_timestamp_us) / 1000000.0f;
        }

        bool reinitialize = !filter_initialized || dt <= 0.0f || dt > 1.5f;
        if (!reinitialize) {
            const float jump = hypotf(forward_m - filtered_forward_m,
                                      left_m - filtered_left_m);
            const float maximum_jump = UWB_MAX_TARGET_SPEED_MPS * dt +
                                       UWB_JUMP_MARGIN_M;
            if (jump > maximum_jump &&
                consecutive_outliers < UWB_REINIT_OUTLIERS) {
                consecutive_outliers++;
                continue;
            }
            if (consecutive_outliers >= UWB_REINIT_OUTLIERS) {
                reinitialize = true;
            }
        }

        const float previous_bearing = filtered_bearing_rad;
        if (reinitialize) {
            filtered_forward_m = forward_m;
            filtered_left_m = left_m;
        } else {
            const float alpha = clampf_local(dt / (UWB_SMOOTH_TAU_S + dt),
                                             0.25f, 0.95f);
            filtered_forward_m += alpha * (forward_m - filtered_forward_m);
            filtered_left_m += alpha * (left_m - filtered_left_m);
        }
        filtered_bearing_rad = atan2f(filtered_left_m, filtered_forward_m);
        const float bearing_rate_rps = filter_initialized && dt > 0.001f
                                           ? fa_wrap_pi(filtered_bearing_rad -
                                                        previous_bearing) / dt
                                           : 0.0f;
        filter_initialized = true;
        consecutive_outliers = 0;
        accepted_timestamp_us = timestamp_us;

        sensors_lock();
        s_sensors.target_distance_m = hypotf(filtered_forward_m,
                                              filtered_left_m);
        s_sensors.target_bearing_rad = filtered_bearing_rad;
        s_sensors.target_bearing_rate_rps = bearing_rate_rps;
        s_sensors.target_timestamp_us = timestamp_us;
        sensors_unlock();
    }
}

static float lidar_body_angle_rad(float raw_degrees)
{
    float relative = -(raw_degrees - (float)CONFIG_FOLLOW_ROBOT_LIDAR_FORWARD_DEG);
    while (relative > 180.0f) relative -= 360.0f;
    while (relative < -180.0f) relative += 360.0f;
    return DEG_TO_RAD(relative);
}

static float pressure_resistance_kohm(float voltage_v)
{
    if (voltage_v <= 0.0f) {
        return INFINITY;
    }
    if (voltage_v >= PRESSURE_EXCITATION_V) {
        return 0.0f;
    }
    return PRESSURE_DIVIDER_KOHM *
           (PRESSURE_EXCITATION_V / voltage_v - 1.0f);
}

static float pressure_force_kg(float resistance_kohm)
{
    if (!isfinite(resistance_kohm) ||
        resistance_kohm >= PRESSURE_NO_LOAD_KOHM) {
        return 0.0f;
    }
    if (resistance_kohm <= 0.0f) {
        return PRESSURE_MAX_KG;
    }

    const float conductance_per_kohm = 1.0f / resistance_kohm;
    float force_kg =
        (conductance_per_kohm - PRESSURE_CONDUCTANCE_OFFSET) /
        PRESSURE_CONDUCTANCE_SLOPE;
    if (force_kg < 2.0f) {
        return 0.0f;
    }
    return fminf(force_kg, PRESSURE_MAX_KG);
}

static void lidar_task(void *argument)
{
    rplidar_c1_t *lidar = (rplidar_c1_t *)argument;
    fa_obstacle_field_t working;
    uint32_t points_since_yield = 0;
    fa_obstacle_reset(&working, LIDAR_SECTORS, LIDAR_FOV_RAD);
    while (true) {
        rplidar_c1_point_t point = {0};
        if (!rplidar_c1_read_point(lidar, &point)) {
            vTaskDelay(1);
            continue;
        }

        if (++points_since_yield >= 32) {
            points_since_yield = 0;
            vTaskDelay(1);
        }

        const bool angle_valid =
            point.angle_deg >= 0.0f && point.angle_deg < 360.0f;
        const bool distance_valid =
            point.distance_mm > 0.0f && point.distance_mm <= 12000.0f;
        const bool quality_valid = point.quality > 0;

        if (!angle_valid || !distance_valid || !quality_valid) {
            continue;
        }

        if (point.start_bit) {
            sensors_lock();
            s_sensors.field = working;
            s_sensors.field_timestamp_us = now_us();
            sensors_unlock();
            fa_obstacle_reset(&working, LIDAR_SECTORS, LIDAR_FOV_RAD);
        }
        const bool in_front = point.angle_deg <= LIDAR_FRONT_MAX_DEG;
        const bool in_other_front = point.angle_deg >= LIDAR_FRONT_MIN_DEG;
        if (in_front || in_other_front) {
            fa_obstacle_add(&working, lidar_body_angle_rad(point.angle_deg),
                            point.distance_mm / 1000.0f);
        }
    }
}

static void ultrasonic_task(void *argument)
{
    ultrasonic_task_arg_t *task = (ultrasonic_task_arg_t *)argument;
    while (true) {
        a02yyuw_reading_t reading = {0};
        esp_err_t ret = a02yyuw_read_dev(task->device, &reading, 0);
        if (ret == ESP_OK && reading.valid) {
            const uint64_t timestamp_us = now_us();
            sensors_lock();
            if (task->left) {
                s_sensors.ultrasonic_left_m =
                    (float)reading.distance_mm / 1000.0f;
                s_sensors.ultrasonic_left_timestamp_us = timestamp_us;
                s_sensors.ultrasonic_left_frames++;
            } else {
                s_sensors.ultrasonic_right_m =
                    (float)reading.distance_mm / 1000.0f;
                s_sensors.ultrasonic_right_timestamp_us = timestamp_us;
                s_sensors.ultrasonic_right_frames++;
            }
            sensors_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void fsr_task(void *argument)
{
    (void)argument;
    unsigned pressure_log_divider = 0;
    while (true) {
        fsr_adc_reading_t reading = {0};
        battery_adc_reading_t battery = {0};
        esp_err_t fsr_ret = fsr_adc_read(&reading);
        esp_err_t battery_ret = battery_adc_read(&battery);
        if ((fsr_ret == ESP_OK && reading.valid) ||
            (battery_ret == ESP_OK && battery.valid)) {
            sensors_lock();
            const uint64_t timestamp_us = now_us();
            if (fsr_ret == ESP_OK && reading.valid) {
                const float resistance_kohm =
                    pressure_resistance_kohm(reading.voltage_v);
                const float force_kg = pressure_force_kg(resistance_kohm);
                s_sensors.fsr_raw = reading.raw;
                s_sensors.fsr_voltage_v = reading.voltage_v;
                s_sensors.fsr_weight_kg =
                    SUITCASE_BASE_WEIGHT_KG + force_kg;
                s_sensors.fsr_timestamp_us = timestamp_us;
                if (++pressure_log_divider >= 2U) {
                    pressure_log_divider = 0;
                    ESP_LOGI("pressure",
                             "force=%.2fkg total=%.2fkg raw=%d adc=%.3fV sensor_R=%.1fkOhm",
                             force_kg, s_sensors.fsr_weight_kg, reading.raw,
                             reading.voltage_v, resistance_kohm);
                }
            }
            if (battery_ret == ESP_OK && battery.valid) {
                s_sensors.battery_raw = battery.raw;
                s_sensors.battery_adc_voltage_v = battery.adc_voltage_v;
                s_sensors.battery_voltage_v = battery.battery_voltage_v;
                s_sensors.battery_percent = battery.percent;
                s_sensors.battery_timestamp_us = timestamp_us;
            }
            sensors_unlock();
        } else {
            if (fsr_ret != ESP_OK) {
                ESP_LOGW(TAG, "FSR read failed: %s", esp_err_to_name(fsr_ret));
            }
            if (battery_ret != ESP_OK) {
                ESP_LOGW(TAG, "battery read failed: %s", esp_err_to_name(battery_ret));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static fa_config_t follow_config(void)
{
    fa_config_t config = fa_default_config();
    config.follow_distance_m = CONFIG_FOLLOW_ROBOT_FOLLOW_DISTANCE_MM / 1000.0f;
    config.stop_band_m = CONFIG_FOLLOW_ROBOT_STOP_BAND_MM / 1000.0f;
    config.max_linear_mps = CONFIG_FOLLOW_ROBOT_MAX_LINEAR_MMPS / 1000.0f;
    config.max_angular_rps = CONFIG_FOLLOW_ROBOT_MAX_ANGULAR_MRADPS / 1000.0f;
    config.emergency_distance_m = CONFIG_FOLLOW_ROBOT_EMERGENCY_DIST_MM / 1000.0f;
    config.slow_distance_m = CONFIG_FOLLOW_ROBOT_SLOW_DIST_MM / 1000.0f;
    config.safe_distance_m = CONFIG_FOLLOW_ROBOT_SAFE_DIST_MM / 1000.0f;
    config.robot_half_width_m = CONFIG_FOLLOW_ROBOT_ROBOT_HALF_WIDTH_MM / 1000.0f;
    return config;
}

static float telemetry_clearance(float distance_m)
{
    return distance_m >= FA_NO_OBSTACLE * 0.5f
               ? LIDAR_TELEMETRY_MAX_M
               : clampf_local(distance_m, 0.0f, LIDAR_TELEMETRY_MAX_M);
}

static chassis_config_t chassis_config(void)
{
    chassis_config_t config = chassis_default_config();
    config.left_invert = false;
    config.right_invert = false;
    config.left_enc_invert = false;
    config.right_enc_invert = false;
    config.ticks_per_meter = (float)CONFIG_FOLLOW_ROBOT_TICKS_PER_METER;
    config.track_width_m = CONFIG_FOLLOW_ROBOT_TRACK_WIDTH_MM / 1000.0f;
    config.max_speed_mps = CONFIG_FOLLOW_ROBOT_MAX_WHEEL_SPEED_MMPS / 1000.0f;
    config.kp = (float)CONFIG_FOLLOW_ROBOT_SPEED_KP;
    config.ki = (float)CONFIG_FOLLOW_ROBOT_SPEED_KI;
    config.kd = (float)CONFIG_FOLLOW_ROBOT_SPEED_KD;
    config.pid_out_limit_us = (float)CONFIG_FOLLOW_ROBOT_SPEED_PID_LIMIT_US;
    config.encoder_stall_timeout_s = 0.0f;
    config.slew_us_per_s = 900.0f;
    return config;
}

static fsr_adc_config_t fsr_config(void)
{
    fsr_adc_config_t config = fsr_adc_default_config();
    config.sample_count = CONFIG_FOLLOW_ROBOT_FSR_SAMPLE_COUNT;
    config.reference_voltage_v = (float)CONFIG_FOLLOW_ROBOT_FSR_REFERENCE_MV / 1000.0f;
    config.calibration.slope_v_per_kg =
        (float)CONFIG_FOLLOW_ROBOT_FSR_SLOPE_UV_PER_KG / 1000000.0f;
    config.calibration.offset_v =
        (float)CONFIG_FOLLOW_ROBOT_FSR_OFFSET_UV / 1000000.0f;
    config.calibration.min_kg =
        (float)CONFIG_FOLLOW_ROBOT_FSR_MIN_KG / 100.0f;
    config.calibration.max_kg =
        (float)CONFIG_FOLLOW_ROBOT_FSR_MAX_KG / 100.0f;
    return config;
}

static const char *follow_state_name(fa_state_t state)
{
    switch (state) {
    case FA_STATE_IDLE: return "IDLE";
    case FA_STATE_SEARCH: return "SEARCH";
    case FA_STATE_FOLLOW: return "FOLLOW";
    case FA_STATE_AVOID: return "AVOID";
    case FA_STATE_ESTOP: return "OBSTACLE_STOP";
    default: return "UNKNOWN";
    }
}

static void control_task(void *argument)
{
    chassis_t *chassis = (chassis_t *)argument;
    fa_ctx_t follow;
    fa_init(&follow, NULL);
    follow.cfg = follow_config();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ROBOT_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t previous_us = now_us();
    unsigned log_counter = 0;

    while (true) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t current_us = now_us();
        float dt = (float)(current_us - previous_us) / 1000000.0f;
        previous_us = current_us;
        if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

        fa_target_t target = {0};
        fa_obstacle_field_t field;
        fa_range_t ultrasonic_left = {0};
        fa_range_t ultrasonic_right = {0};
        uint32_t ultrasonic_left_frames;
        uint32_t ultrasonic_right_frames;
        float fsr_voltage_v;
        float fsr_weight_kg;
        int fsr_raw;
        uint64_t fsr_timestamp_us;
        float battery_voltage_v;
        float battery_percent;
        float battery_adc_voltage_v;
        int battery_raw;
        uint64_t battery_timestamp_us;
        sensors_lock();
        target.valid = is_fresh(current_us, s_sensors.target_timestamp_us,
                                TARGET_FRESH_US);
        target.distance_m = s_sensors.target_distance_m;
        target.bearing_rad = s_sensors.target_bearing_rad;
        target.bearing_rate_rps = s_sensors.target_bearing_rate_rps;
        const bool have_field = is_fresh(current_us, s_sensors.field_timestamp_us,
                                         FIELD_FRESH_US);
        field = s_sensors.field;
        ultrasonic_left.valid = is_fresh(
            current_us, s_sensors.ultrasonic_left_timestamp_us, ULTRA_FRESH_US);
        ultrasonic_left.dist_m = s_sensors.ultrasonic_left_m;
        ultrasonic_right.valid = is_fresh(
            current_us, s_sensors.ultrasonic_right_timestamp_us, ULTRA_FRESH_US);
        ultrasonic_right.dist_m = s_sensors.ultrasonic_right_m;
        ultrasonic_left_frames = s_sensors.ultrasonic_left_frames;
        ultrasonic_right_frames = s_sensors.ultrasonic_right_frames;
        fsr_voltage_v = s_sensors.fsr_voltage_v;
        fsr_weight_kg = s_sensors.fsr_weight_kg;
        fsr_raw = s_sensors.fsr_raw;
        fsr_timestamp_us = s_sensors.fsr_timestamp_us;
        battery_voltage_v = s_sensors.battery_voltage_v;
        battery_percent = s_sensors.battery_percent;
        battery_adc_voltage_v = s_sensors.battery_adc_voltage_v;
        battery_raw = s_sensors.battery_raw;
        battery_timestamp_us = s_sensors.battery_timestamp_us;
        sensors_unlock();

        float lidar_front_m = 0.0f;
        if (have_field) {
            lidar_front_m = telemetry_clearance(fa_obstacle_clearance(
                &field, -follow.cfg.front_cone_rad, follow.cfg.front_cone_rad));
        }
        const float combined_front_m = have_field
                                           ? lidar_front_m
                                           : LIDAR_TELEMETRY_MAX_M;
        const float combined_left_m = ultrasonic_left.valid
                                          ? ultrasonic_left.dist_m
                                          : LIDAR_TELEMETRY_MAX_M;
        const float combined_right_m = ultrasonic_right.valid
                                           ? ultrasonic_right.dist_m
                                           : LIDAR_TELEMETRY_MAX_M;

        web_control_command_t web_command = {0};
        web_control_get_command(&web_command);
        fa_output_t output = {0};
        esp_err_t command_ret;
        const char *state_name;
        if (web_command.estop_latched) {
            command_ret = chassis_emergency_stop(chassis);
            state_name = "WEB_ESTOP";
        } else if (web_command.mode == WEB_CONTROL_MODE_MANUAL) {
            if (web_command.client_alive) {
                command_ret = chassis_set_velocity(
                    chassis, web_command.manual_linear_mps,
                    web_command.manual_angular_rps);
                state_name = "MANUAL";
            } else {
                command_ret = chassis_set_velocity(chassis, 0.0f, 0.0f);
                state_name = "MANUAL_TIMEOUT";
            }
        } else {
            output = fa_update(&follow, &target, have_field ? &field : NULL,
                               &ultrasonic_left, &ultrasonic_right, dt);
            output.v_mps *= (float)web_command.follow_speed_pct / 100.0f;
            output.omega_rps *= (float)web_command.follow_turn_pct / 100.0f;
            command_ret = chassis_set_velocity(chassis, output.v_mps,
                                               output.omega_rps);
            state_name = follow_state_name(output.state);
        }
        esp_err_t update_ret = chassis_update(chassis, dt);
        if (command_ret != ESP_OK || update_ret != ESP_OK) {
            esp_err_t stop_ret = chassis_emergency_stop(chassis);
            ESP_LOGE(TAG, "chassis command=%s update=%s emergency=%s",
                     esp_err_to_name(command_ret), esp_err_to_name(update_ret),
                     esp_err_to_name(stop_ret));
            state_name = "CHASSIS_FAULT";
        }

        float measured_v = 0.0f;
        float measured_w = 0.0f;
        chassis_get_measured(chassis, &measured_v, &measured_w, NULL, NULL);
        web_control_telemetry_t telemetry = {
            .state = state_name,
            .uwb_ok = target.valid,
            .lidar_ok = have_field,
            .ultrasonic_left_ok = ultrasonic_left.valid,
            .ultrasonic_right_ok = ultrasonic_right.valid,
            .fsr_ok = is_fresh(current_us, fsr_timestamp_us, FSR_FRESH_US),
            .battery_ok = is_fresh(current_us, battery_timestamp_us,
                                   BATTERY_FRESH_US),
            .encoder_ok = !chassis_encoder_faulted(chassis),
            .target_distance_m = target.distance_m,
            .target_bearing_rad = target.bearing_rad,
            .front_clearance_m = combined_front_m,
            .left_clearance_m = combined_left_m,
            .right_clearance_m = combined_right_m,
            .chosen_heading_rad = output.chosen_heading_rad,
            .fsr_voltage_v = fsr_voltage_v,
            .fsr_weight_kg = fsr_weight_kg,
            .fsr_raw = fsr_raw,
            .battery_voltage_v = battery_voltage_v,
            .battery_percent = battery_percent,
            .battery_adc_voltage_v = battery_adc_voltage_v,
            .battery_raw = battery_raw,
            .measured_linear_mps = measured_v,
            .measured_angular_rps = measured_w,
            .left_pulse_us = (int)chassis->cmd_pulse_l_us,
            .right_pulse_us = (int)chassis->cmd_pulse_r_us,
        };
        web_control_publish(&telemetry);

        if (++log_counter >= (unsigned)CONFIG_FOLLOW_ROBOT_CONTROL_HZ) {
            log_counter = 0;
            ESP_LOGI(TAG, "%s target=%d d=%.2f bearing=%+.2f cmd=%+.2f/%+.2f pulses=%d/%d | lidar=%d ultra=%d/%d fsr=%d raw=%d adc=%.3fV weight=%.2fkg frames=%lu/%lu rmt_evt=%lu/%lu sym=%u/%u bytes=%lu/%lu front=%.2f left=%.2f right=%.2f heading=%+.1fdeg",
                     state_name, target.valid, target.distance_m,
                     target.bearing_rad, output.v_mps, output.omega_rps,
                     telemetry.left_pulse_us, telemetry.right_pulse_us,
                     have_field, ultrasonic_left.valid, ultrasonic_right.valid,
                     telemetry.fsr_ok, telemetry.fsr_raw,
                     telemetry.fsr_voltage_v, telemetry.fsr_weight_kg,
                     (unsigned long)ultrasonic_left_frames,
                     (unsigned long)ultrasonic_right_frames,
                     (unsigned long)s_ultrasonic_left.sw_uart.rx_events,
                     (unsigned long)s_ultrasonic_right.sw_uart.rx_events,
                     (unsigned)s_ultrasonic_left.sw_uart.last_symbol_count,
                     (unsigned)s_ultrasonic_right.sw_uart.last_symbol_count,
                     (unsigned long)s_ultrasonic_left.sw_uart.decoded_bytes,
                     (unsigned long)s_ultrasonic_right.sw_uart.decoded_bytes,
                     combined_front_m, combined_left_m, combined_right_m,
                     output.chosen_heading_rad * 180.0f / PI_F);
        }
    }
}

static bool start_task(TaskFunction_t function, const char *name,
                       uint32_t stack_size, void *argument,
                       UBaseType_t priority)
{
    if (xTaskCreate(function, name, stack_size, argument, priority, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed: %s", name);
        return false;
    }
    return true;
}

static bool start_task_on_core(TaskFunction_t function, const char *name,
                               uint32_t stack_size, void *argument,
                               UBaseType_t priority, BaseType_t core_id)
{
    if (xTaskCreatePinnedToCore(function, name, stack_size, argument, priority,
                                NULL, core_id) != pdPASS) {
        ESP_LOGE(TAG, "task create failed: %s", name);
        return false;
    }
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Follow control starting: filtered UWB + lidar clearance + slew-limited ESC output");
    imu_i2c_scan();
    memset(&s_sensors, 0, sizeof(s_sensors));
    s_sensors.lock = xSemaphoreCreateMutex();
    if (s_sensors.lock == NULL) {
        ESP_LOGE(TAG, "sensor mutex allocation failed");
        return;
    }
    fa_obstacle_reset(&s_sensors.field, LIDAR_SECTORS, LIDAR_FOV_RAD);

    chassis_config_t drive = chassis_config();
    esp_err_t ret = chassis_init(&s_chassis, &drive);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "chassis init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = chassis_emergency_stop(&s_chassis);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "neutral output failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESC neutral %dus; arming for %dms",
             ESC_PWM_NEUTRAL_US, ESC_ARM_TIME_MS);
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME_MS));

    bu_uwb_config_t uwb = bu_uwb_default_config(UWB_UART_PORT,
                                                PIN_UWB_RX, PIN_UWB_TX);
    uwb.baudrate = UWB_BAUD_RATE;
    if (uwb.rx_buffer_size < 4096) {
        uwb.rx_buffer_size = 4096;
    }
    ret = bu_uwb_init(&uwb);
    if (ret == ESP_OK) {
        start_task(uwb_task, "uwb", 4096, NULL, 6);
    } else {
        ESP_LOGE(TAG, "UWB init failed: %s", esp_err_to_name(ret));
    }

    rplidar_c1_config_t lidar = rplidar_c1_default_config(
        RPLIDAR_UART_PORT, PIN_RPLIDAR_RX, PIN_RPLIDAR_TX);
    lidar.baudrate = RPLIDAR_BAUD_RATE;
    if (lidar.rx_buffer_size < 4096) {
        lidar.rx_buffer_size = 4096;
    }
    ret = rplidar_c1_init(&s_lidar, &lidar);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RPLIDAR init failed: %s", esp_err_to_name(ret));
        goto skip_lidar;
    }
    ESP_LOGI(TAG, "RPLIDAR UART initialized: UART%d, TX=%d, RX=%d, baud=%d",
             (int)RPLIDAR_UART_PORT, PIN_RPLIDAR_TX, PIN_RPLIDAR_RX, lidar.baudrate);

    rplidar_c1_stop(&s_lidar);
    rplidar_c1_reset(&s_lidar);
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t health_status = 0;
    uint16_t health_error = 0;
    ret = rplidar_c1_get_health(&s_lidar, &health_status, &health_error);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RPLIDAR health: status=%u, error_code=0x%04X",
                 (unsigned)health_status, (unsigned)health_error);
        if (health_status == 2) {
            ESP_LOGE(TAG, "RPLIDAR health error, aborting lidar");
            rplidar_c1_deinit(&s_lidar);
            goto skip_lidar;
        }
    } else {
        ESP_LOGW(TAG, "RPLIDAR health check failed: %s", esp_err_to_name(ret));
    }

    rplidar_c1_info_t lidar_info = {0};
    ret = rplidar_c1_get_info(&s_lidar, &lidar_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RPLIDAR device: model=%u.%u, fw=%u.%u, hw=%u, SN=%s",
                 (unsigned)lidar_info.major_model, (unsigned)lidar_info.sub_model,
                 (unsigned)lidar_info.firmware_major, (unsigned)lidar_info.firmware_minor,
                 (unsigned)lidar_info.hardware, lidar_info.serial_num);
    } else {
        ESP_LOGW(TAG, "RPLIDAR device info unavailable: %s", esp_err_to_name(ret));
    }

    ret = rplidar_c1_set_motor_speed(&s_lidar, LIDAR_MOTOR_RPM);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RPLIDAR motor speed set to %d RPM", LIDAR_MOTOR_RPM);
        vTaskDelay(pdMS_TO_TICKS(300));
    } else {
        ESP_LOGW(TAG, "RPLIDAR motor speed command failed: %s",
                 esp_err_to_name(ret));
    }

    ret = rplidar_c1_start_scan(&s_lidar);
    if (ret == ESP_OK) {
        start_task_on_core(lidar_task, "lidar", 4096, &s_lidar, 6, 1);
    } else {
        ESP_LOGE(TAG, "RPLIDAR start scan failed: %s", esp_err_to_name(ret));
        rplidar_c1_deinit(&s_lidar);
    }
skip_lidar:;

    a02yyuw_config_t ultrasonic_left = a02yyuw_default_config(
        UART_NUM_0, PIN_ULTRASONIC_LEFT_RX, -1);
    ultrasonic_left.use_sw_uart = true;
    ultrasonic_left.baudrate = ULTRASONIC_BAUD_RATE;
    ret = a02yyuw_init_dev(&s_ultrasonic_left, &ultrasonic_left);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "left ultrasonic ready: SW UART RX=GPIO%d baud=%d",
                 PIN_ULTRASONIC_LEFT_RX, ULTRASONIC_BAUD_RATE);
    } else {
        ESP_LOGE(TAG, "left ultrasonic init failed: %s", esp_err_to_name(ret));
    }

    a02yyuw_config_t ultrasonic_right = a02yyuw_default_config(
        UART_NUM_0, PIN_ULTRASONIC_RIGHT_RX, -1);
    ultrasonic_right.use_sw_uart = true;
    ultrasonic_right.baudrate = ULTRASONIC_BAUD_RATE;
    ret = a02yyuw_init_dev(&s_ultrasonic_right, &ultrasonic_right);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "right ultrasonic ready: SW UART RX=GPIO%d baud=%d",
                 PIN_ULTRASONIC_RIGHT_RX, ULTRASONIC_BAUD_RATE);
    } else {
        ESP_LOGE(TAG, "right ultrasonic init failed: %s", esp_err_to_name(ret));
    }

    if (s_ultrasonic_left.initialized && s_ultrasonic_right.initialized) {
        ESP_LOGI(TAG, "ultrasonic RX: dual parallel RMT capture");
        start_task(ultrasonic_task, "ultra_left", 3072,
                   &s_ultrasonic_left_arg, 5);
        start_task(ultrasonic_task, "ultra_right", 3072,
                   &s_ultrasonic_right_arg, 5);
    } else if (s_ultrasonic_left.initialized) {
        start_task(ultrasonic_task, "ultra_left", 3072,
                   &s_ultrasonic_left_arg, 5);
    } else if (s_ultrasonic_right.initialized) {
        start_task(ultrasonic_task, "ultra_right", 3072,
                   &s_ultrasonic_right_arg, 5);
    }

    fsr_adc_config_t fsr = fsr_config();
    ESP_LOGI(TAG, "FSR config: GPIO%d ch=%d samples=%d ref=%.2fV slope=%.1fuV/kg offset=%.1fuV clamp=[%.1f,%.1f]kg",
             fsr.adc_gpio, (int)fsr.adc_channel, fsr.sample_count,
             fsr.reference_voltage_v,
             fsr.calibration.slope_v_per_kg * 1000000.0f,
             fsr.calibration.offset_v * 1000000.0f,
             fsr.calibration.min_kg, fsr.calibration.max_kg);
    ret = fsr_adc_init(&fsr);
    if (ret == ESP_OK) {
        start_task(fsr_task, "fsr", 3072, NULL, 3);
    } else {
        ESP_LOGE(TAG, "FSR init failed: %s", esp_err_to_name(ret));
    }

    web_control_config_t web = web_control_default_config();
    web.ap_ssid = CONFIG_FOLLOW_ROBOT_WEB_AP_SSID;
    web.ap_password = CONFIG_FOLLOW_ROBOT_WEB_AP_PASSWORD;
    web.command_timeout_ms = CONFIG_FOLLOW_ROBOT_WEB_TIMEOUT_MS;
    ret = web_control_init(&web);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "web control init failed: %s; motion remains stopped",
                 esp_err_to_name(ret));
    }

    ret = ble_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE control init failed: %s", esp_err_to_name(ret));
    }

    if (!start_task(control_task, "control", 6144, &s_chassis, 7)) {
        chassis_emergency_stop(&s_chassis);
    }
}
