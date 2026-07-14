#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"

#define SW_UART_MAX_RX_BUF 1024

typedef struct {
    int rx_gpio;
    int tx_gpio;
    int baudrate;
    int rx_buffer_size;
} sw_uart_config_t;

typedef enum {
    SW_UART_IDLE,
    SW_UART_START,
    SW_UART_DATA,
    SW_UART_STOP
} sw_uart_state_t;

typedef struct sw_uart_inst {
    int rx_gpio;
    int tx_gpio;
    int baudrate;
    uint8_t rx_buf[SW_UART_MAX_RX_BUF];
    volatile int rx_head;
    volatile int rx_tail;
    volatile sw_uart_state_t state;
    volatile uint8_t current_byte;
    volatile int bit_count;
    
    gptimer_handle_t timer;
    portMUX_TYPE lock;
    
    uint32_t bit_us;
    uint32_t half_bit_us;
    bool initialized;
} sw_uart_t;

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio);
esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config);
int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms);
void sw_uart_flush(sw_uart_t *uart);
void sw_uart_deinit(sw_uart_t *uart);
