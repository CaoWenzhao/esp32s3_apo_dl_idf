#include "sw_uart.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/task.h"

static const char *TAG = "sw_uart";

static bool IRAM_ATTR sw_uart_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    sw_uart_t *uart = (sw_uart_t *)user_ctx;
    int level = gpio_get_level(uart->rx_gpio);
    bool yield = false;

    switch (uart->state) {
        case SW_UART_START:
            if (level == 0) {
                uart->current_byte = 0;
                uart->bit_count = 0;
                uart->state = SW_UART_DATA;
                
                gptimer_alarm_config_t alarm_config = {
                    .alarm_count = edata->alarm_value + uart->bit_us,
                };
                gptimer_set_alarm_action(timer, &alarm_config);
            } else {
                uart->state = SW_UART_IDLE;
                gptimer_stop(timer);
                gpio_intr_enable(uart->rx_gpio);
            }
            break;
            
        case SW_UART_DATA:
            uart->current_byte >>= 1;
            if (level) {
                uart->current_byte |= 0x80;
            }
            uart->bit_count++;
            
            if (uart->bit_count >= 8) {
                uart->state = SW_UART_STOP;
            }
            
            gptimer_alarm_config_t alarm_config_data = {
                .alarm_count = edata->alarm_value + uart->bit_us,
            };
            gptimer_set_alarm_action(timer, &alarm_config_data);
            break;
            
        case SW_UART_STOP:
            if (level) {
                portENTER_CRITICAL_ISR(&uart->lock);
                int next_head = uart->rx_head + 1;
                if (next_head >= SW_UART_MAX_RX_BUF) {
                    next_head = 0;
                }
                
                if (next_head != uart->rx_tail) {
                    uart->rx_buf[uart->rx_head] = uart->current_byte;
                    uart->rx_head = next_head;
                }
                portEXIT_CRITICAL_ISR(&uart->lock);
            }
            uart->state = SW_UART_IDLE;
            gptimer_stop(timer);
            gpio_intr_enable(uart->rx_gpio);
            break;
            
        default:
            uart->state = SW_UART_IDLE;
            gptimer_stop(timer);
            gpio_intr_enable(uart->rx_gpio);
            break;
    }
    return yield;
}

static void IRAM_ATTR sw_uart_gpio_isr(void *arg)
{
    sw_uart_t *uart = (sw_uart_t *)arg;
    
    if (gpio_get_level(uart->rx_gpio) != 0) return;
    
    gpio_intr_disable(uart->rx_gpio);
    uart->state = SW_UART_START;
    gptimer_set_raw_count(uart->timer, 0);
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = uart->half_bit_us,
        .flags.auto_reload_on_alarm = false,
    };
    
    gptimer_set_alarm_action(uart->timer, &alarm_config);
    gptimer_start(uart->timer);
}

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio)
{
    sw_uart_config_t config = {
        .rx_gpio = rx_gpio, 
        .tx_gpio = tx_gpio,
        .baudrate = 9600, 
        .rx_buffer_size = SW_UART_MAX_RX_BUF,
    };
    return config;
}

esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config)
{
    if (!uart || !config || config->rx_gpio < 0) return ESP_ERR_INVALID_ARG;
    
    memset(uart, 0, sizeof(*uart));
    portMUX_TYPE init_lock = portMUX_INITIALIZER_UNLOCKED;
    uart->lock = init_lock;
    uart->rx_gpio = config->rx_gpio;
    uart->tx_gpio = config->tx_gpio;
    uart->baudrate = config->baudrate > 0 ? config->baudrate : 9600;
    uart->state = SW_UART_IDLE;
    
    uart->bit_us = 1000000 / uart->baudrate;
    if (uart->bit_us < 1) uart->bit_us = 1;
    
    uart->half_bit_us = uart->bit_us / 2;
    if (uart->half_bit_us < 1) uart->half_bit_us = 1;
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    
    esp_err_t ret = gptimer_new_timer(&timer_config, &uart->timer);
    if (ret != ESP_OK) return ret;
    
    gptimer_event_callbacks_t cbs = { .on_alarm = sw_uart_timer_cb };
    ret = gptimer_register_event_callbacks(uart->timer, &cbs, uart);
    if (ret != ESP_OK) goto err_timer;
    
    ret = gptimer_enable(uart->timer);
    if (ret != ESP_OK) goto err_timer;
    
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << uart->rx_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    
    ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) goto err_timer;
    
    gpio_install_isr_service(0);
    ret = gpio_isr_handler_add(uart->rx_gpio, sw_uart_gpio_isr, uart);
    if (ret != ESP_OK) goto err_timer;
    
    uart->initialized = true;
    return ESP_OK;

err_timer:
    gptimer_del_timer(uart->timer);
    return ret;
}

int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    if (!uart || !buf || max_len <= 0) return 0;
    uint32_t waited = 0;
    
    while (waited <= timeout_ms) {
        portENTER_CRITICAL(&uart->lock);
        int tail = uart->rx_tail, head = uart->rx_head;
        portEXIT_CRITICAL(&uart->lock);
        
        int avail = (head >= tail) ? (head - tail) : (SW_UART_MAX_RX_BUF - tail + head);
        if (avail > 0) {
            int to_read = avail < max_len ? avail : max_len;
            for (int i = 0; i < to_read; i++) {
                buf[i] = uart->rx_buf[tail];
                tail = (tail + 1) % SW_UART_MAX_RX_BUF;
            }
            
            portENTER_CRITICAL(&uart->lock);
            uart->rx_tail = tail;
            portEXIT_CRITICAL(&uart->lock);
            
            return to_read;
        }
        
        if (timeout_ms == 0) break;
        vTaskDelay(pdMS_TO_TICKS(1));
        waited++;
    }
    return 0;
}

void sw_uart_flush(sw_uart_t *uart)
{
    if (!uart) return;
    portENTER_CRITICAL(&uart->lock);
    uart->rx_head = uart->rx_tail = 0;
    portEXIT_CRITICAL(&uart->lock);
}

void sw_uart_deinit(sw_uart_t *uart)
{
    if (!uart || !uart->initialized) return;
    gpio_isr_handler_remove(uart->rx_gpio);
    gptimer_disable(uart->timer);
    gptimer_del_timer(uart->timer);
    gpio_reset_pin(uart->rx_gpio);
    uart->initialized = false;
}