#include "rplidar_c1.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RPLIDAR_START_BYTE 0xA5
#define RPLIDAR_CMD_STOP 0x25
#define RPLIDAR_CMD_RESET 0x40
#define RPLIDAR_CMD_SCAN 0x20
#define RPLIDAR_CMD_GET_HEALTH 0x52
#define RPLIDAR_CMD_GET_INFO 0x50
#define RPLIDAR_CMD_MOTOR_SPEED 0xA8
#define RPLIDAR_RESP_GET_INFO 0x04
#define RPLIDAR_RESP_GET_HEALTH 0x06
#define RPLIDAR_RESP_SCAN 0x81

static bool check_descriptor(const uint8_t *resp,
                             uint32_t expected_len,
                             uint8_t expected_type)
{
    const uint32_t len = ((uint32_t)resp[2]) |
                         ((uint32_t)resp[3] << 8) |
                         ((uint32_t)resp[4] << 16) |
                         (((uint32_t)resp[5] & 0x3F) << 24);

    return resp[0] == 0xA5 &&
           resp[1] == 0x5A &&
           len == expected_len &&
           resp[6] == expected_type;
}

rplidar_c1_config_t rplidar_c1_default_config(uart_port_t uart_port,
                                              int rx_gpio,
                                              int tx_gpio)
{
    rplidar_c1_config_t config = {
        .uart_port = uart_port,
        .tx_gpio = tx_gpio,
        .rx_gpio = rx_gpio,
        .baudrate = RPLIDAR_C1_DEFAULT_BAUDRATE,
        .rx_buffer_size = RPLIDAR_C1_DEFAULT_RX_BUF_SIZE,
    };
    return config;
}

static esp_err_t send_cmd(rplidar_c1_t *lidar, uint8_t cmd)
{
    if (lidar == NULL || !lidar->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t packet[2] = {RPLIDAR_START_BYTE, cmd};
    return uart_write_bytes(lidar->config.uart_port,
                            packet,
                            sizeof(packet)) == sizeof(packet)
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t rplidar_c1_init(rplidar_c1_t *lidar,
                          const rplidar_c1_config_t *config)
{
    if (lidar == NULL || config == NULL || config->rx_gpio < 0 || config->tx_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(lidar, 0, sizeof(*lidar));
    lidar->config = *config;
    if (lidar->config.baudrate <= 0) {
        lidar->config.baudrate = RPLIDAR_C1_DEFAULT_BAUDRATE;
    }
    if (lidar->config.rx_buffer_size <= 0) {
        lidar->config.rx_buffer_size = RPLIDAR_C1_DEFAULT_RX_BUF_SIZE;
    }

    const uart_config_t uart_config = {
        .baud_rate = lidar->config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(lidar->config.uart_port, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_set_pin(lidar->config.uart_port,
                       lidar->config.tx_gpio,
                       lidar->config.rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_driver_install(lidar->config.uart_port,
                              lidar->config.rx_buffer_size,
                              0,
                              0,
                              NULL,
                              0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    lidar->initialized = true;
    return ESP_OK;
}

void rplidar_c1_deinit(rplidar_c1_t *lidar)
{
    if (lidar != NULL && lidar->initialized) {
        uart_driver_delete(lidar->config.uart_port);
        lidar->initialized = false;
    }
}

void rplidar_c1_stop(rplidar_c1_t *lidar)
{
    if (send_cmd(lidar, RPLIDAR_CMD_STOP) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        uart_flush(lidar->config.uart_port);
    }
}

void rplidar_c1_reset(rplidar_c1_t *lidar)
{
    if (send_cmd(lidar, RPLIDAR_CMD_RESET) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(600));
        uart_flush(lidar->config.uart_port);
        lidar->parser_state = 0;
    }
}

esp_err_t rplidar_c1_get_health(rplidar_c1_t *lidar,
                                uint8_t *status,
                                uint16_t *error_code)
{
    if (lidar == NULL || status == NULL || error_code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    rplidar_c1_stop(lidar);
    esp_err_t ret = send_cmd(lidar, RPLIDAR_CMD_GET_HEALTH);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t resp[10] = {0};
    const int len = uart_read_bytes(lidar->config.uart_port,
                                    resp,
                                    sizeof(resp),
                                    pdMS_TO_TICKS(100));
    if (len < (int)sizeof(resp)) {
        return ESP_ERR_TIMEOUT;
    }
    if (!check_descriptor(resp, 3, RPLIDAR_RESP_GET_HEALTH)) {
        return ESP_FAIL;
    }

    *status = resp[7];
    *error_code = ((uint16_t)resp[9] << 8) | resp[8];
    return ESP_OK;
}

esp_err_t rplidar_c1_get_info(rplidar_c1_t *lidar,
                              rplidar_c1_info_t *info)
{
    if (lidar == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    rplidar_c1_stop(lidar);
    esp_err_t ret = send_cmd(lidar, RPLIDAR_CMD_GET_INFO);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t resp[27] = {0};
    const int len = uart_read_bytes(lidar->config.uart_port,
                                    resp,
                                    sizeof(resp),
                                    pdMS_TO_TICKS(100));
    if (len < (int)sizeof(resp)) {
        return ESP_ERR_TIMEOUT;
    }
    if (!check_descriptor(resp, 20, RPLIDAR_RESP_GET_INFO)) {
        return ESP_FAIL;
    }

    info->major_model = resp[7];
    info->sub_model = resp[8];
    info->firmware_minor = resp[9];
    info->firmware_major = resp[10];
    info->hardware = resp[11];
    for (int i = 0; i < 16; ++i) {
        snprintf(&info->serial_num[i * 2], 3, "%02X", resp[12 + i]);
    }
    info->serial_num[32] = '\0';
    return ESP_OK;
}

esp_err_t rplidar_c1_set_motor_speed(rplidar_c1_t *lidar, uint16_t rpm)
{
    if (lidar == NULL || !lidar->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t packet[6] = {
        RPLIDAR_START_BYTE,
        RPLIDAR_CMD_MOTOR_SPEED,
        0x02,
        (uint8_t)(rpm & 0xFF),
        (uint8_t)(rpm >> 8),
        0,
    };
    for (int i = 0; i < 5; ++i) {
        packet[5] ^= packet[i];
    }
    return uart_write_bytes(lidar->config.uart_port,
                            packet,
                            sizeof(packet)) == sizeof(packet)
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t rplidar_c1_start_scan(rplidar_c1_t *lidar)
{
    if (lidar == NULL || !lidar->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uart_flush(lidar->config.uart_port);
    esp_err_t ret = send_cmd(lidar, RPLIDAR_CMD_SCAN);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t resp[7] = {0};
    const int len = uart_read_bytes(lidar->config.uart_port,
                                    resp,
                                    sizeof(resp),
                                    pdMS_TO_TICKS(500));
    if (len < (int)sizeof(resp) ||
        !check_descriptor(resp, 5, RPLIDAR_RESP_SCAN)) {
        return ESP_FAIL;
    }
    lidar->parser_state = 0;
    return ESP_OK;
}

bool rplidar_c1_read_point(rplidar_c1_t *lidar, rplidar_c1_point_t *point)
{
    if (lidar == NULL || point == NULL || !lidar->initialized) {
        return false;
    }

    uint8_t ch = 0;
    while (uart_read_bytes(lidar->config.uart_port, &ch, 1, 0) > 0) {
        switch (lidar->parser_state) {
        case 0:
            if (((ch ^ (ch >> 1)) & 0x01) == 1) {
                lidar->parser_buf[0] = ch;
                lidar->parser_state = 1;
            }
            break;
        case 1:
            if ((ch & 0x01) == 1) {
                lidar->parser_buf[1] = ch;
                lidar->parser_state = 2;
            } else {
                lidar->parser_state = 0;
                if (((ch ^ (ch >> 1)) & 0x01) == 1) {
                    lidar->parser_buf[0] = ch;
                    lidar->parser_state = 1;
                }
            }
            break;
        case 2:
            lidar->parser_buf[2] = ch;
            lidar->parser_state = 3;
            break;
        case 3:
            lidar->parser_buf[3] = ch;
            lidar->parser_state = 4;
            break;
        default:
            lidar->parser_buf[4] = ch;
            lidar->parser_state = 0;
            point->start_bit = (lidar->parser_buf[0] & 0x01) != 0;
            point->quality = lidar->parser_buf[0] >> 2;
            const uint16_t angle_q6 =
                ((uint16_t)lidar->parser_buf[2] << 7) |
                (lidar->parser_buf[1] >> 1);
            const uint16_t distance_q2 =
                ((uint16_t)lidar->parser_buf[4] << 8) |
                lidar->parser_buf[3];
            point->angle_deg = (float)angle_q6 / 64.0f;
            point->distance_mm = (float)distance_q2 / 4.0f;
            return true;
        }
    }
    return false;
}
