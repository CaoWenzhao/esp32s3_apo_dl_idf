#include "bu_uwb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bu_uwb";
static bu_uwb_config_t s_config;
static bool s_initialized;

bu_uwb_config_t bu_uwb_default_config(uart_port_t uart_port,
                                      int rx_gpio,
                                      int tx_gpio)
{
    bu_uwb_config_t config = {
        .uart_port = uart_port,
        .tx_gpio = tx_gpio,
        .rx_gpio = rx_gpio,
        .baudrate = BU_UWB_DEFAULT_BAUDRATE,
        .rx_buffer_size = BU_UWB_DEFAULT_RX_BUF_SIZE,
    };
    return config;
}

bool bu_uwb_parse_distance_line(const char *line, bu_uwb_distance_t *out)
{
    if (out != NULL) {
        out->distance_m = 0.0f;
        out->distance_mm = 0;
        out->valid = false;
    }
    if (line == NULL || out == NULL) {
        return false;
    }

    const char *p = strstr(line, "distance:");
    if (p == NULL) {
        p = strstr(line, "distance=");
    }
    if (p == NULL) {
        p = strstr(line, "dist:");
    }
    if (p == NULL) {
        p = strstr(line, "dist=");
    }

    if (p == NULL) {
        return false;
    }

    const char *colon = strchr(p, ':');
    const char *equals = strchr(p, '=');
    if (colon == NULL) {
        p = equals;
    } else if (equals == NULL) {
        p = colon;
    } else {
        p = colon < equals ? colon : equals;
    }
    if (p == NULL) {
        return false;
    }
    p++;

    char *end = NULL;
    const float distance_m = strtof(p, &end);
    if (end == p || distance_m < 0.0f) {
        return false;
    }

    out->distance_m = distance_m;
    out->distance_mm = (int)(distance_m * 1000.0f + 0.5f);
    out->valid = true;
    return true;
}

static bool extract_json_string(const char *line,
                                const char *key,
                                char *out,
                                size_t out_size)
{
    if (line == NULL || key == NULL || out == NULL || out_size == 0) {
        return false;
    }

    char pattern[24] = {0};
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(line, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);
    const char *end = strchr(p, '"');
    if (end == NULL || end == p) {
        return false;
    }

    size_t len = (size_t)(end - p);
    if (len >= out_size) {
        len = out_size - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool extract_json_int(const char *line, const char *key, int *out)
{
    if (line == NULL || key == NULL || out == NULL) {
        return false;
    }

    char pattern[24] = {0};
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(line, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);

    char *end = NULL;
    const long value = strtol(p, &end, 10);
    if (end == p) {
        return false;
    }
    while (*end == ' ') {
        end++;
    }
    if (*end != ',' && *end != '}') {
        return false;
    }
    *out = (int)value;
    return true;
}

bool bu_uwb_parse_twr_line(const char *line, bu_uwb_twr_reading_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    if (line == NULL || out == NULL) {
        return false;
    }
    if (strncmp(line, "JS", 2) != 0 || strstr(line, "\"TWR\"") == NULL) {
        return false;
    }

    const char *json_start = strchr(line, '{');
    if (json_start == NULL || json_start <= line + 2) {
        return false;
    }

    size_t frame_len = (size_t)(json_start - line);
    if (frame_len >= sizeof(out->frame_id)) {
        frame_len = sizeof(out->frame_id) - 1;
    }
    memcpy(out->frame_id, line, frame_len);
    out->frame_id[frame_len] = '\0';

    if (!extract_json_string(line, "a16", out->anchor_id, sizeof(out->anchor_id)) ||
        !extract_json_int(line, "R", &out->r) ||
        !extract_json_int(line, "T", &out->timestamp) ||
        !extract_json_int(line, "D", &out->distance_cm) ||
        !extract_json_int(line, "P", &out->p) ||
        !extract_json_int(line, "Xcm", &out->x_cm) ||
        !extract_json_int(line, "Ycm", &out->y_cm) ||
        !extract_json_int(line, "O", &out->orientation) ||
        !extract_json_int(line, "V", &out->validity) ||
        !extract_json_int(line, "X", &out->x) ||
        !extract_json_int(line, "Y", &out->y) ||
        !extract_json_int(line, "Z", &out->z)) {
        memset(out, 0, sizeof(*out));
        return false;
    }

    out->valid = true;
    return true;
}

bu_uwb_line_type_t bu_uwb_classify_line(const char *line)
{
    if (line == NULL) {
        return BU_UWB_LINE_UNKNOWN;
    }
    if (strncmp(line, "DATA,", 5) == 0) {
        return BU_UWB_LINE_DATA;
    }
    if (strncmp(line, "ERR,", 4) == 0) {
        return BU_UWB_LINE_ERROR;
    }
    if (strncmp(line, "JS", 2) == 0 && strstr(line, "\"TWR\"") != NULL) {
        return BU_UWB_LINE_TWR;
    }
    return BU_UWB_LINE_UNKNOWN;
}

const char *bu_uwb_line_payload(const char *line)
{
    if (line == NULL) {
        return NULL;
    }
    switch (bu_uwb_classify_line(line)) {
    case BU_UWB_LINE_DATA:
        return line + 5;
    case BU_UWB_LINE_ERROR:
        return line + 4;
    default:
        return line;
    }
}

esp_err_t bu_uwb_init(const bu_uwb_config_t *config)
{
    if (config == NULL || config->rx_gpio < 0 || config->tx_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    if (s_config.baudrate <= 0) {
        s_config.baudrate = BU_UWB_DEFAULT_BAUDRATE;
    }
    if (s_config.rx_buffer_size <= 0) {
        s_config.rx_buffer_size = BU_UWB_DEFAULT_RX_BUF_SIZE;
    }

    const uart_config_t uart_config = {
        .baud_rate = s_config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(s_config.uart_port, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_set_pin(s_config.uart_port,
                       s_config.tx_gpio,
                       s_config.rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_driver_install(s_config.uart_port,
                              s_config.rx_buffer_size,
                              0,
                              0,
                              NULL,
                              0);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_initialized = true;
        ESP_LOGI(TAG, "UART%d RX=GPIO%d TX=GPIO%d baud=%d",
                 s_config.uart_port,
                 s_config.rx_gpio,
                 s_config.tx_gpio,
                 s_config.baudrate);
        return ESP_OK;
    }
    return ret;
}

esp_err_t bu_uwb_send_command(const char *command)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_write_bytes(s_config.uart_port, command, strlen(command));
    if (strstr(command, "\r\n") == NULL) {
        uart_write_bytes(s_config.uart_port, "\r\n", 2);
    }
    return ESP_OK;
}

esp_err_t bu_uwb_read_bytes(uint8_t *data,
                            size_t data_size,
                            int *out_len,
                            uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || data_size == 0 || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int got = uart_read_bytes(s_config.uart_port,
                                    data,
                                    data_size,
                                    pdMS_TO_TICKS(timeout_ms));
    *out_len = got;
    return got > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t bu_uwb_read_line(char *line, size_t line_size, uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (line == NULL || line_size < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t pos = 0;
    line[0] = '\0';
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1000 : timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        uint8_t ch = 0;
        const int got = uart_read_bytes(s_config.uart_port, &ch, 1, pdMS_TO_TICKS(20));
        if (got <= 0) {
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (pos == 0) {
                continue;
            }
            line[pos] = '\0';
            return ESP_OK;
        }
        if (pos + 1 < line_size) {
            line[pos++] = (char)ch;
        } else {
            pos = 0;
            line[0] = '\0';
            return ESP_ERR_INVALID_SIZE;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t bu_uwb_request_distance(bu_uwb_distance_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->distance_m = 0.0f;
    out->distance_mm = 0;
    out->valid = false;

    char line[BU_UWB_LINE_MAX] = {0};
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1000 : timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        const esp_err_t ret = bu_uwb_read_line(line, sizeof(line), 100);
        if (ret == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (ret != ESP_OK) {
            return ret;
        }
        if (bu_uwb_parse_distance_line(line, out)) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

void bu_uwb_deinit(void)
{
    if (s_initialized) {
        uart_driver_delete(s_config.uart_port);
        s_initialized = false;
    }
}
