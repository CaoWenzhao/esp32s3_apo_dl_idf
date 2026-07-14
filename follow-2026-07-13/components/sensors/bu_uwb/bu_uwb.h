#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "bu_uwb_parser.h"

#define BU_UWB_DEFAULT_BAUDRATE 115200
#define BU_UWB_DEFAULT_RX_BUF_SIZE 1024

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baudrate;
    int rx_buffer_size;
} bu_uwb_config_t;


bu_uwb_config_t bu_uwb_default_config(uart_port_t uart_port,
                                      int rx_gpio,
                                      int tx_gpio);
esp_err_t bu_uwb_init(const bu_uwb_config_t *config);
esp_err_t bu_uwb_send_command(const char *command);
esp_err_t bu_uwb_read_bytes(uint8_t *data,
                            size_t data_size,
                            int *out_len,
                            uint32_t timeout_ms);
esp_err_t bu_uwb_read_line(char *line, size_t line_size, uint32_t timeout_ms);
esp_err_t bu_uwb_request_distance(bu_uwb_distance_t *out, uint32_t timeout_ms);
void bu_uwb_deinit(void);

