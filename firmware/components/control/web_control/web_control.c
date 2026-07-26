#include "web_control.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    SemaphoreHandle_t lock;
    web_control_config_t config;
    web_control_command_t command;
    web_control_telemetry_t telemetry;
    uint64_t last_command_us;
} control_state_t;

static control_state_t s_state;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void state_lock(void)
{
    xSemaphoreTake(s_state.lock, portMAX_DELAY);
}

static void state_unlock(void)
{
    xSemaphoreGive(s_state.lock);
}

static void mark_command_received(void)
{
    s_state.last_command_us = now_us();
    s_state.command.client_alive = true;
}

web_control_config_t web_control_default_config(void)
{
    const web_control_config_t config = {
        .command_timeout_ms = 1000,
        .max_manual_linear_mps = 0.8f,
        .max_manual_angular_rps = 1.6f,
        .default_follow_speed_pct = 100,
        .default_follow_turn_pct = 100,
        .default_remote_speed_pct = 100,
    };
    return config;
}

esp_err_t web_control_init(const web_control_config_t *config)
{
    if (config == NULL || config->command_timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.lock = xSemaphoreCreateMutex();
    if (s_state.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state.config = *config;
    s_state.command.mode = WEB_CONTROL_MODE_AUTO;
    s_state.command.estop_latched = true;
    s_state.command.follow_speed_pct = config->default_follow_speed_pct <= 100
                                           ? config->default_follow_speed_pct
                                           : 100;
    s_state.command.follow_turn_pct = config->default_follow_turn_pct <= 100
                                          ? config->default_follow_turn_pct
                                          : 100;
    s_state.command.remote_speed_pct = config->default_remote_speed_pct <= 100
                                           ? config->default_remote_speed_pct
                                           : 100;
    return ESP_OK;
}

void web_control_heartbeat(void)
{
    state_lock();
    mark_command_received();
    state_unlock();
}

void web_control_emergency_stop(void)
{
    state_lock();
    s_state.command.estop_latched = true;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
}

void web_control_arm(void)
{
    state_lock();
    s_state.command.estop_latched = false;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
}

esp_err_t web_control_set_mode(web_control_mode_t mode)
{
    if (mode != WEB_CONTROL_MODE_AUTO && mode != WEB_CONTROL_MODE_MANUAL) {
        return ESP_ERR_INVALID_ARG;
    }
    state_lock();
    s_state.command.mode = mode;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

esp_err_t web_control_set_manual(float linear, float angular)
{
    state_lock();
    if (s_state.command.mode != WEB_CONTROL_MODE_MANUAL) {
        state_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const float scale = (float)s_state.command.remote_speed_pct / 100.0f;
    s_state.command.manual_linear_mps =
        clampf(linear, -1.0f, 1.0f) *
        s_state.config.max_manual_linear_mps * scale;
    s_state.command.manual_angular_rps =
        clampf(angular, -1.0f, 1.0f) *
        s_state.config.max_manual_angular_rps * scale;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

esp_err_t web_control_set_speeds(uint8_t follow_speed, uint8_t follow_turn,
                                 uint8_t remote_speed)
{
    if (follow_speed > 100 || follow_turn > 100 || remote_speed > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    state_lock();
    s_state.command.follow_speed_pct = follow_speed;
    s_state.command.follow_turn_pct = follow_turn;
    s_state.command.remote_speed_pct = remote_speed;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

void web_control_get_command(web_control_command_t *command)
{
    if (command == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    const uint64_t current_us = now_us();
    const uint64_t age_us = current_us >= s_state.last_command_us
                                ? current_us - s_state.last_command_us
                                : UINT64_MAX;
    const uint64_t timeout_us =
        (uint64_t)s_state.config.command_timeout_ms * 1000ULL;
    if (s_state.last_command_us == 0 || age_us > timeout_us) {
        s_state.command.client_alive = false;
        s_state.command.manual_linear_mps = 0.0f;
        s_state.command.manual_angular_rps = 0.0f;
    }
    s_state.command.command_age_ms = age_us == UINT64_MAX
                                         ? UINT32_MAX
                                         : (uint32_t)(age_us / 1000ULL);
    *command = s_state.command;
    state_unlock();
}

void web_control_publish(const web_control_telemetry_t *telemetry)
{
    if (telemetry == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    s_state.telemetry = *telemetry;
    state_unlock();
}

void web_control_get_snapshot(web_control_command_t *command,
                              web_control_telemetry_t *telemetry)
{
    if (command != NULL) {
        web_control_get_command(command);
    }
    if (telemetry != NULL && s_state.lock != NULL) {
        state_lock();
        *telemetry = s_state.telemetry;
        state_unlock();
    }
}
