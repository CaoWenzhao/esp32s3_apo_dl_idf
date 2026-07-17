#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SW_UART_MAX_RX_BUF 1024
#define SW_UART_RMT_SYMBOLS 64

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
    rmt_channel_handle_t rx_channel;
    QueueHandle_t rx_queue;
    rmt_symbol_word_t symbols[SW_UART_RMT_SYMBOLS];
    rmt_receive_config_t receive_config;
    uint32_t bit_us;
    bool initialized;
    volatile bool rx_enabled;
    volatile bool receiving;
    volatile uint32_t rx_events;
    volatile uint32_t decoded_bytes;
    volatile size_t last_symbol_count;
} sw_uart_t;

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio);
esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config);
int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms);
esp_err_t sw_uart_set_rx_enabled(sw_uart_t *uart, bool enabled);
void sw_uart_flush(sw_uart_t *uart);
void sw_uart_deinit(sw_uart_t *uart);
