#include "a02yyuw.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "a02yyuw";
/* Backing device for the legacy single-instance API. */
static a02yyuw_t s_singleton;

a02yyuw_config_t a02yyuw_default_config(uart_port_t uart_port,
                                        int rx_gpio,
                                        int tx_gpio)
{
    a02yyuw_config_t config = {
        .uart_port = uart_port,
        .tx_gpio = tx_gpio,
        .rx_gpio = rx_gpio,
        .baudrate = A02YYUW_DEFAULT_BAUDRATE,
        .rx_buffer_size = A02YYUW_DEFAULT_RX_BUF_SIZE,
        .use_sw_uart = false,
    };
    return config;
}

bool a02yyuw_parse_frame(const uint8_t *frame,
                         size_t len,
                         a02yyuw_reading_t *out)
{
    if (out != NULL) {
        out->distance_mm = 0;
        out->valid = false;
    }
    if (frame == NULL || out == NULL || len < 4 || frame[0] != 0xFF) {
        return false;
    }

    const uint8_t checksum = (uint8_t)((frame[0] + frame[1] + frame[2]) & 0xFF);
    if (checksum != frame[3]) {
        return false;
    }

    const uint16_t distance = ((uint16_t)frame[1] << 8) | frame[2];
    if (distance < A02YYUW_MIN_DISTANCE_MM ||
        distance > A02YYUW_MAX_DISTANCE_MM) {
        return false;
    }

    out->distance_mm = (int)distance;
    out->valid = true;
    return true;
}

bool a02yyuw_parse_latest(const uint8_t *buf,
                          size_t len,
                          a02yyuw_reading_t *out)
{
    bool found = false;
    a02yyuw_reading_t last = {0};

    if (buf == NULL || out == NULL || len < 4) {
        if (out != NULL) {
            out->distance_mm = 0;
            out->valid = false;
        }
        return false;
    }

    for (size_t i = 0; i + 4 <= len; ++i) {
        a02yyuw_reading_t current = {0};
        if (a02yyuw_parse_frame(&buf[i], 4, &current)) {
            last = current;
            found = true;
        }
    }

    *out = last;
    return found;
}

esp_err_t a02yyuw_init_dev(a02yyuw_t *dev, const a02yyuw_config_t *config)
{
    if (dev == NULL || config == NULL || config->rx_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->config = *config;
    if (dev->config.baudrate <= 0) {
        dev->config.baudrate = A02YYUW_DEFAULT_BAUDRATE;
    }
    if (dev->config.rx_buffer_size <= 0) {
        dev->config.rx_buffer_size = A02YYUW_DEFAULT_RX_BUF_SIZE;
    }

    dev->use_sw_uart = config->use_sw_uart;
    dev->initialized = false;

    if (dev->use_sw_uart) {
        sw_uart_config_t sw_cfg = sw_uart_default_config(
            dev->config.rx_gpio, dev->config.tx_gpio);
        sw_cfg.baudrate = dev->config.baudrate;
        esp_err_t ret = sw_uart_init(&dev->sw_uart, &sw_cfg);
        if (ret == ESP_OK) {
            dev->initialized = true;
            ESP_LOGI(TAG, "SW-UART RX=GPIO%d baud=%d",
                     dev->config.rx_gpio,
                     dev->config.baudrate);
        }
        return ret;
    }

    const uart_config_t uart_config = {
        .baud_rate = dev->config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(dev->config.uart_port, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }

    /* A02YYUW is transmit-only; leave the ESP TX line unrouted when tx_gpio<0. */
    const int tx_pin = dev->config.tx_gpio < 0 ? UART_PIN_NO_CHANGE
                                               : dev->config.tx_gpio;
    ret = uart_set_pin(dev->config.uart_port,
                       tx_pin,
                       dev->config.rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = uart_driver_install(dev->config.uart_port,
                              dev->config.rx_buffer_size,
                              0,
                              0,
                              NULL,
                              0);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        dev->initialized = true;
        ESP_LOGI(TAG, "UART%d RX=GPIO%d TX=GPIO%d baud=%d",
                 dev->config.uart_port,
                 dev->config.rx_gpio,
                 dev->config.tx_gpio,
                 dev->config.baudrate);
        return ESP_OK;
    }
    return ret;
}

esp_err_t a02yyuw_read_dev(a02yyuw_t *dev,
                           a02yyuw_reading_t *out,
                           uint32_t wait_ms)
{
    if (dev == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->distance_mm = 0;
    out->valid = false;

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (wait_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    uint8_t buf[128] = {0};
    int len;

    if (dev->use_sw_uart) {
        len = sw_uart_read_bytes(&dev->sw_uart, buf, sizeof(buf), 50);
    } else {
        len = uart_read_bytes(dev->config.uart_port,
                              buf,
                              sizeof(buf),
                              pdMS_TO_TICKS(50));
    }

    if (len < 4) {
        return ESP_ERR_TIMEOUT;
    }

    return a02yyuw_parse_latest(buf, (size_t)len, out) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

void a02yyuw_deinit_dev(a02yyuw_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }
    if (dev->use_sw_uart) {
        sw_uart_deinit(&dev->sw_uart);
    } else {
        uart_driver_delete(dev->config.uart_port);
    }
    dev->initialized = false;
}

/* Legacy single-instance API: thin wrappers over the static backing device. */
esp_err_t a02yyuw_init(const a02yyuw_config_t *config)
{
    return a02yyuw_init_dev(&s_singleton, config);
}

esp_err_t a02yyuw_read(a02yyuw_reading_t *out, uint32_t wait_ms)
{
    return a02yyuw_read_dev(&s_singleton, out, wait_ms);
}

void a02yyuw_deinit(void)
{
    a02yyuw_deinit_dev(&s_singleton);
}
