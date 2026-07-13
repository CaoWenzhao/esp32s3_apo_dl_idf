#include "bu_uwb.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bu_uwb";
static bu_uwb_config_t s_config;
static bool s_initialized;

bu_uwb_config_t bu_uwb_default_config(uart_port_t uart_port, int rx_gpio,
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

esp_err_t bu_uwb_init(const bu_uwb_config_t *config)
{
    if (config == NULL || config->rx_gpio < 0 || config->tx_gpio < 0)
        return ESP_ERR_INVALID_ARG;
    s_config = *config;
    if (s_config.baudrate <= 0) s_config.baudrate = BU_UWB_DEFAULT_BAUDRATE;
    if (s_config.rx_buffer_size <= 0)
        s_config.rx_buffer_size = BU_UWB_DEFAULT_RX_BUF_SIZE;
    const uart_config_t uart_config = {
        .baud_rate = s_config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(s_config.uart_port, &uart_config);
    if (ret != ESP_OK) return ret;
    ret = uart_set_pin(s_config.uart_port, s_config.tx_gpio, s_config.rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;
    ret = uart_driver_install(s_config.uart_port, s_config.rx_buffer_size, 0,
                              0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    s_initialized = true;
    ESP_LOGI(TAG, "UART%d RX=GPIO%d TX=GPIO%d baud=%d", s_config.uart_port,
             s_config.rx_gpio, s_config.tx_gpio, s_config.baudrate);
    return ESP_OK;
}

esp_err_t bu_uwb_send_command(const char *command)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (command == NULL) return ESP_ERR_INVALID_ARG;
    const size_t length = strlen(command);
    if (uart_write_bytes(s_config.uart_port, command, length) != (int)length)
        return ESP_FAIL;
    if (strstr(command, "\r\n") == NULL &&
        uart_write_bytes(s_config.uart_port, "\r\n", 2) != 2)
        return ESP_FAIL;
    return ESP_OK;
}

esp_err_t bu_uwb_read_bytes(uint8_t *data, size_t data_size, int *out_len,
                            uint32_t timeout_ms)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (data == NULL || data_size == 0 || out_len == NULL)
        return ESP_ERR_INVALID_ARG;
    const int received = uart_read_bytes(s_config.uart_port, data, data_size,
                                         pdMS_TO_TICKS(timeout_ms));
    *out_len = received;
    return received > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t bu_uwb_read_line(char *line, size_t line_size, uint32_t timeout_ms)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (line == NULL || line_size < 2) return ESP_ERR_INVALID_ARG;
    size_t position = 0;
    line[0] = '\0';
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms == 0 ? 1000 : timeout_ms);
    while (xTaskGetTickCount() - start < timeout) {
        uint8_t character = 0;
        const int received = uart_read_bytes(s_config.uart_port, &character, 1,
                                             pdMS_TO_TICKS(20));
        if (received <= 0 || character == '\r') continue;
        if (character == '\n') {
            if (position == 0) continue;
            line[position] = '\0';
            return ESP_OK;
        }
        if (position + 1 >= line_size) {
            line[0] = '\0';
            return ESP_ERR_INVALID_SIZE;
        }
        line[position++] = (char)character;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t bu_uwb_request_distance(bu_uwb_distance_t *out, uint32_t timeout_ms)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    *out = (bu_uwb_distance_t){0};
    char line[BU_UWB_LINE_MAX];
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms == 0 ? 1000 : timeout_ms);
    while (xTaskGetTickCount() - start < timeout) {
        esp_err_t ret = bu_uwb_read_line(line, sizeof(line), 100);
        if (ret == ESP_ERR_TIMEOUT) continue;
        if (ret != ESP_OK) return ret;
        if (bu_uwb_parse_distance_line(line, out)) return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

void bu_uwb_deinit(void)
{
    if (s_initialized) {
        esp_err_t ret = uart_driver_delete(s_config.uart_port);
        if (ret != ESP_OK) ESP_LOGE(TAG, "UART delete failed: %s", esp_err_to_name(ret));
        s_initialized = false;
    }
}
