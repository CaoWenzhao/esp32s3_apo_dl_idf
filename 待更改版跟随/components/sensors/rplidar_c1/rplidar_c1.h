#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#define RPLIDAR_C1_DEFAULT_BAUDRATE 460800
#define RPLIDAR_C1_DEFAULT_RX_BUF_SIZE 4096

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baudrate;
    int rx_buffer_size;
} rplidar_c1_config_t;

typedef struct {
    float angle_deg;
    float distance_mm;
    uint8_t quality;
    bool start_bit;
} rplidar_c1_point_t;

typedef struct {
    uint8_t major_model;
    uint8_t sub_model;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware;
    char serial_num[33];
} rplidar_c1_info_t;

typedef struct {
    rplidar_c1_config_t config;
    uint8_t parser_buf[5];
    int parser_state;
    bool initialized;
} rplidar_c1_t;

rplidar_c1_config_t rplidar_c1_default_config(uart_port_t uart_port,
                                              int rx_gpio,
                                              int tx_gpio);
esp_err_t rplidar_c1_init(rplidar_c1_t *lidar,
                          const rplidar_c1_config_t *config);
void rplidar_c1_deinit(rplidar_c1_t *lidar);
void rplidar_c1_stop(rplidar_c1_t *lidar);
void rplidar_c1_reset(rplidar_c1_t *lidar);
esp_err_t rplidar_c1_get_health(rplidar_c1_t *lidar,
                                uint8_t *status,
                                uint16_t *error_code);
esp_err_t rplidar_c1_get_info(rplidar_c1_t *lidar,
                              rplidar_c1_info_t *info);
esp_err_t rplidar_c1_set_motor_speed(rplidar_c1_t *lidar, uint16_t rpm);
esp_err_t rplidar_c1_start_scan(rplidar_c1_t *lidar);
bool rplidar_c1_read_point(rplidar_c1_t *lidar, rplidar_c1_point_t *point);

