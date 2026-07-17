#include "sw_uart.h"

#include <string.h>

#include "freertos/task.h"

typedef struct {
    uint8_t level;
    uint32_t start_us;
    uint32_t end_us;
} uart_segment_t;

static bool IRAM_ATTR sw_uart_rx_done(rmt_channel_handle_t channel,
                                      const rmt_rx_done_event_data_t *edata,
                                      void *user_ctx)
{
    (void)channel;
    sw_uart_t *uart = (sw_uart_t *)user_ctx;
    BaseType_t wake = pdFALSE;
    size_t symbol_count = edata->num_symbols;
    uart->receiving = false;
    uart->rx_events++;
    uart->last_symbol_count = symbol_count;
    xQueueOverwriteFromISR(uart->rx_queue, &symbol_count, &wake);
    return wake == pdTRUE;
}

static esp_err_t sw_uart_start_receive(sw_uart_t *uart)
{
    if (!uart->rx_enabled || uart->receiving) return ESP_OK;
    esp_err_t ret = rmt_receive(uart->rx_channel, uart->symbols,
                                sizeof(uart->symbols),
                                &uart->receive_config);
    if (ret == ESP_OK) uart->receiving = true;
    return ret;
}

static int segment_level_at(const uart_segment_t *segments, int segment_count,
                            uint32_t sample_us)
{
    for (int i = 0; i < segment_count; ++i) {
        if (sample_us >= segments[i].start_us &&
            sample_us < segments[i].end_us) {
            return segments[i].level;
        }
    }
    return -1;
}

static int decode_uart(const sw_uart_t *uart, size_t symbol_count,
                       uint8_t *output, int output_size)
{
    uart_segment_t segments[SW_UART_RMT_SYMBOLS * 2];
    int segment_count = 0;
    uint32_t cursor_us = 0;
    if (symbol_count > SW_UART_RMT_SYMBOLS) {
        symbol_count = SW_UART_RMT_SYMBOLS;
    }

    for (size_t i = 0; i < symbol_count; ++i) {
        const rmt_symbol_word_t symbol = uart->symbols[i];
        if (symbol.duration0 > 0) {
            segments[segment_count++] = (uart_segment_t) {
                .level = symbol.level0,
                .start_us = cursor_us,
                .end_us = cursor_us + symbol.duration0,
            };
            cursor_us += symbol.duration0;
        }
        if (symbol.duration1 > 0) {
            segments[segment_count++] = (uart_segment_t) {
                .level = symbol.level1,
                .start_us = cursor_us,
                .end_us = cursor_us + symbol.duration1,
            };
            cursor_us += symbol.duration1;
        }
    }

    if (segment_count < (int)(SW_UART_RMT_SYMBOLS * 2)) {
        segments[segment_count++] = (uart_segment_t) {
            .level = 1,
            .start_us = cursor_us,
            .end_us = cursor_us + 3000,
        };
    }

    int output_count = 0;
    uint32_t ignore_before_us = 0;
    for (int i = 0; i < segment_count && output_count < output_size; ++i) {
        const bool falling_edge = segments[i].level == 0 &&
                                  (i == 0 || segments[i - 1].level == 1);
        const uint32_t start_us = segments[i].start_us;
        if (!falling_edge || start_us < ignore_before_us) continue;

        uint8_t value = 0;
        bool complete = true;
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t sample_us = start_us +
                (uint32_t)((3 + 2 * bit) * uart->bit_us / 2);
            int level = segment_level_at(segments, segment_count, sample_us);
            if (level < 0) {
                complete = false;
                break;
            }
            if (level) value |= (uint8_t)(1U << bit);
        }

        const uint32_t stop_sample_us = start_us +
            (uint32_t)(19 * uart->bit_us / 2);
        if (complete &&
            segment_level_at(segments, segment_count, stop_sample_us) == 1) {
            output[output_count++] = value;
            ignore_before_us = start_us + 10 * uart->bit_us;
        }
    }
    return output_count;
}

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio)
{
    return (sw_uart_config_t) {
        .rx_gpio = rx_gpio,
        .tx_gpio = tx_gpio,
        .baudrate = 9600,
        .rx_buffer_size = SW_UART_MAX_RX_BUF,
    };
}

esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config)
{
    if (!uart || !config || config->rx_gpio < 0) return ESP_ERR_INVALID_ARG;

    memset(uart, 0, sizeof(*uart));
    uart->rx_gpio = config->rx_gpio;
    uart->tx_gpio = config->tx_gpio;
    uart->baudrate = config->baudrate > 0 ? config->baudrate : 9600;
    uart->bit_us = 1000000U / (uint32_t)uart->baudrate;
    uart->rx_queue = xQueueCreate(1, sizeof(size_t));
    if (!uart->rx_queue) return ESP_ERR_NO_MEM;

    rmt_rx_channel_config_t channel_config = {
        .gpio_num = uart->rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = SW_UART_RMT_SYMBOLS,
        .intr_priority = 0,
    };
    esp_err_t ret = rmt_new_rx_channel(&channel_config, &uart->rx_channel);
    if (ret != ESP_OK) goto fail_queue;

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = sw_uart_rx_done,
    };
    ret = rmt_rx_register_event_callbacks(uart->rx_channel, &callbacks, uart);
    if (ret != ESP_OK) goto fail_channel;

    ret = rmt_enable(uart->rx_channel);
    if (ret != ESP_OK) goto fail_channel;

    uart->receive_config = (rmt_receive_config_t) {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 3000000,
    };
    uart->rx_enabled = true;
    uart->initialized = true;
    ret = sw_uart_start_receive(uart);
    if (ret == ESP_OK) return ESP_OK;

    uart->initialized = false;
    rmt_disable(uart->rx_channel);
fail_channel:
    rmt_del_channel(uart->rx_channel);
fail_queue:
    vQueueDelete(uart->rx_queue);
    return ret;
}

int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len,
                       uint32_t timeout_ms)
{
    if (!uart || !uart->initialized || !buf || max_len <= 0) return 0;

    size_t symbol_count = 0;
    if (xQueueReceive(uart->rx_queue, &symbol_count,
                      pdMS_TO_TICKS(timeout_ms)) != pdPASS) {
        return 0;
    }

    int decoded = decode_uart(uart, symbol_count, buf, max_len);
    uart->decoded_bytes += (uint32_t)decoded;
    sw_uart_start_receive(uart);
    return decoded;
}

esp_err_t sw_uart_set_rx_enabled(sw_uart_t *uart, bool enabled)
{
    if (!uart || !uart->initialized) return ESP_ERR_INVALID_STATE;
    if (uart->rx_enabled == enabled) return ESP_OK;

    uart->rx_enabled = enabled;
    uart->receiving = false;
    xQueueReset(uart->rx_queue);
    if (!enabled) return rmt_disable(uart->rx_channel);

    esp_err_t ret = rmt_enable(uart->rx_channel);
    if (ret != ESP_OK) return ret;
    return sw_uart_start_receive(uart);
}

void sw_uart_flush(sw_uart_t *uart)
{
    if (uart && uart->rx_queue) xQueueReset(uart->rx_queue);
}

void sw_uart_deinit(sw_uart_t *uart)
{
    if (!uart || !uart->initialized) return;
    uart->rx_enabled = false;
    uart->initialized = false;
    rmt_disable(uart->rx_channel);
    rmt_del_channel(uart->rx_channel);
    vQueueDelete(uart->rx_queue);
    uart->rx_channel = NULL;
    uart->rx_queue = NULL;
}
