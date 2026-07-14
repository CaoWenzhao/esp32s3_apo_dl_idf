#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "sw_uart.h"

#define A02YYUW_DEFAULT_BAUDRATE 9600
#define A02YYUW_DEFAULT_RX_BUF_SIZE 512
#define A02YYUW_MIN_DISTANCE_MM 30
#define A02YYUW_MAX_DISTANCE_MM 4500

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baudrate;
    int rx_buffer_size;
    bool use_sw_uart;
} a02yyuw_config_t;

typedef struct {
    int distance_mm;
    bool valid;
} a02yyuw_reading_t;

a02yyuw_config_t a02yyuw_default_config(uart_port_t uart_port,
                                        int rx_gpio,
                                        int tx_gpio);
bool a02yyuw_parse_frame(const uint8_t *frame,
                         size_t len,
                         a02yyuw_reading_t *out);
bool a02yyuw_parse_latest(const uint8_t *buf,
                          size_t len,
                          a02yyuw_reading_t *out);

/* --- Single-instance (legacy) API -----------------------------------------
 * Kept for backward compatibility. Internally it drives a single static
 * device, so it can only manage one ultrasonic sensor at a time. New code
 * with two or more sensors should use the handle-based *_dev API below. */
esp_err_t a02yyuw_init(const a02yyuw_config_t *config);
esp_err_t a02yyuw_read(a02yyuw_reading_t *out, uint32_t wait_ms);
void a02yyuw_deinit(void);

/* --- Multi-instance (handle-based) API ------------------------------------
 * Each a02yyuw_t owns its own UART (hardware port or software bit-bang UART),
 * so several A02YYUW sensors can run at the same time, each on its own pin.
 * The A02YYUW transmits autonomously, so only an ESP RX pin is required;
 * set config.tx_gpio < 0 when no TX line is wired. */
typedef struct {
    a02yyuw_config_t config;
    bool initialized;
    bool use_sw_uart;
    sw_uart_t sw_uart;
} a02yyuw_t;

esp_err_t a02yyuw_init_dev(a02yyuw_t *dev, const a02yyuw_config_t *config);
esp_err_t a02yyuw_read_dev(a02yyuw_t *dev,
                           a02yyuw_reading_t *out,
                           uint32_t wait_ms);
void a02yyuw_deinit_dev(a02yyuw_t *dev);

