#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#define BU_UWB_DEFAULT_BAUDRATE 115200
#define BU_UWB_DEFAULT_RX_BUF_SIZE 1024
#define BU_UWB_LINE_MAX 160

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baudrate;
    int rx_buffer_size;
} bu_uwb_config_t;

typedef struct {
    float distance_m;
    int distance_mm;
    bool valid;
} bu_uwb_distance_t;

typedef struct {
    char frame_id[8];
    char anchor_id[8];
    int r;
    int timestamp;
    int distance_cm;
    int p;
    int x_cm;
    int y_cm;
    int orientation;
    int validity;
    int x;
    int y;
    int z;
    bool valid;
} bu_uwb_twr_reading_t;

typedef enum {
    BU_UWB_LINE_UNKNOWN = 0,
    BU_UWB_LINE_DATA,
    BU_UWB_LINE_ERROR,
    BU_UWB_LINE_TWR,
} bu_uwb_line_type_t;

bu_uwb_config_t bu_uwb_default_config(uart_port_t uart_port,
                                      int rx_gpio,
                                      int tx_gpio);
bu_uwb_line_type_t bu_uwb_classify_line(const char *line);
const char *bu_uwb_line_payload(const char *line);
bool bu_uwb_parse_distance_line(const char *line, bu_uwb_distance_t *out);
bool bu_uwb_parse_twr_line(const char *line, bu_uwb_twr_reading_t *out);
esp_err_t bu_uwb_init(const bu_uwb_config_t *config);
esp_err_t bu_uwb_send_command(const char *command);
esp_err_t bu_uwb_read_bytes(uint8_t *data,
                            size_t data_size,
                            int *out_len,
                            uint32_t timeout_ms);
esp_err_t bu_uwb_read_line(char *line, size_t line_size, uint32_t timeout_ms);
esp_err_t bu_uwb_request_distance(bu_uwb_distance_t *out, uint32_t timeout_ms);
void bu_uwb_deinit(void);

