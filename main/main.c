#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#include "esp_err.h"

// ===================== 引脚定义 =====================

// APO-DL A/B 通道 RC 信号输入
#define LEFT_ESC_GPIO       GPIO_NUM_4
#define RIGHT_ESC_GPIO      GPIO_NUM_5

// 编码器输入，不影响电机转动，只用于查看计数
#define LEFT_ENC_A_GPIO     GPIO_NUM_6
#define LEFT_ENC_B_GPIO     GPIO_NUM_7

#define RIGHT_ENC_A_GPIO    GPIO_NUM_15
#define RIGHT_ENC_B_GPIO    GPIO_NUM_16

// 你已经手动把一边电机蓝白线反接了，所以这里都不用软件反向
#define LEFT_MOTOR_REVERSE   0
#define RIGHT_MOTOR_REVERSE  0

// ===================== RC 航模信号参数 =====================

#define ESC_FREQ_HZ          50
#define ESC_PERIOD_US        20000

#define ESC_MIN_US           1000
#define ESC_MID_US           1500
#define ESC_MAX_US           2000

#define LEDC_DUTY_RES        LEDC_TIMER_14_BIT
#define LEDC_DUTY_MAX        ((1 << 14) - 1)

// Start at the full ESC range. Use '-' to reduce the test output.
#define DEFAULT_SPEED_DELTA_US   500
#define MIN_SPEED_DELTA_US       50
#define MAX_SPEED_DELTA_US       500
#define SPEED_STEP_US            25
#define ESC_ARM_DELAY_MS         3000

// ===================== 全局变量 =====================

static volatile int64_t left_count = 0;
static volatile int64_t right_count = 0;

static volatile uint8_t left_last_state = 0;
static volatile uint8_t right_last_state = 0;

static int speed_delta_us = DEFAULT_SPEED_DELTA_US;

static int last_left_pulse = ESC_MID_US;
static int last_right_pulse = ESC_MID_US;
static bool uart_input_ready = false;
static bool usb_input_ready = false;

// 四倍频正交编码器查表
static const int8_t quad_table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

// ===================== 工具函数 =====================

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint32_t pulse_us_to_duty(int pulse_us)
{
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);
    return (uint32_t)((pulse_us * LEDC_DUTY_MAX) / ESC_PERIOD_US);
}

// 反向函数保留，但现在宏都是 0，不会实际反向
static int apply_reverse(int pulse_us, int reverse)
{
    if (!reverse) {
        return pulse_us;
    }

    return ESC_MIN_US + ESC_MAX_US - pulse_us;
}

// ===================== 编码器中断 =====================

static void left_encoder_isr(void *arg)
{
    int a = gpio_get_level(LEFT_ENC_A_GPIO);
    int b = gpio_get_level(LEFT_ENC_B_GPIO);

    uint8_t now = (uint8_t)((a << 1) | b);
    uint8_t index = (uint8_t)((left_last_state << 2) | now);

    left_count += quad_table[index];
    left_last_state = now;
}

static void right_encoder_isr(void *arg)
{
    int a = gpio_get_level(RIGHT_ENC_A_GPIO);
    int b = gpio_get_level(RIGHT_ENC_B_GPIO);

    uint8_t now = (uint8_t)((a << 1) | b);
    uint8_t index = (uint8_t)((right_last_state << 2) | now);

    right_count += quad_table[index];
    right_last_state = now;
}

// ===================== RC 输出 =====================

static void set_left_pulse(int pulse_us)
{
    pulse_us = apply_reverse(pulse_us, LEFT_MOTOR_REVERSE);
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);

    last_left_pulse = pulse_us;

    uint32_t duty = pulse_us_to_duty(pulse_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void set_right_pulse(int pulse_us)
{
    pulse_us = apply_reverse(pulse_us, RIGHT_MOTOR_REVERSE);
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);

    last_right_pulse = pulse_us;

    uint32_t duty = pulse_us_to_duty(pulse_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

static void motor_stop(void)
{
    set_left_pulse(ESC_MID_US);
    set_right_pulse(ESC_MID_US);
}

static void motor_forward(void)
{
    set_left_pulse(ESC_MID_US + speed_delta_us);
    set_right_pulse(ESC_MID_US + speed_delta_us);
}

static void motor_backward(void)
{
    set_left_pulse(ESC_MID_US - speed_delta_us);
    set_right_pulse(ESC_MID_US - speed_delta_us);
}

static void motor_turn_left(void)
{
    set_left_pulse(ESC_MID_US - speed_delta_us);
    set_right_pulse(ESC_MID_US + speed_delta_us);
}

static void motor_turn_right(void)
{
    set_left_pulse(ESC_MID_US + speed_delta_us);
    set_right_pulse(ESC_MID_US - speed_delta_us);
}

static void motor_left_only(int direction)
{
    set_left_pulse(ESC_MID_US + direction * speed_delta_us);
    set_right_pulse(ESC_MID_US);
}

static void motor_right_only(int direction)
{
    set_left_pulse(ESC_MID_US);
    set_right_pulse(ESC_MID_US + direction * speed_delta_us);
}

// ===================== 初始化 =====================

static void esc_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = ESC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t left_channel = {
        .gpio_num = LEFT_ESC_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_us_to_duty(ESC_MID_US),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));

    ledc_channel_config_t right_channel = {
        .gpio_num = RIGHT_ESC_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_us_to_duty(ESC_MID_US),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));

    motor_stop();
}

static void encoder_init(void)
{
    gpio_config_t enc_conf = {
        .pin_bit_mask =
            (1ULL << LEFT_ENC_A_GPIO) |
            (1ULL << LEFT_ENC_B_GPIO) |
            (1ULL << RIGHT_ENC_A_GPIO) |
            (1ULL << RIGHT_ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    ESP_ERROR_CHECK(gpio_config(&enc_conf));

    left_last_state = (uint8_t)((gpio_get_level(LEFT_ENC_A_GPIO) << 1) |
                                gpio_get_level(LEFT_ENC_B_GPIO));

    right_last_state = (uint8_t)((gpio_get_level(RIGHT_ENC_A_GPIO) << 1) |
                                 gpio_get_level(RIGHT_ENC_B_GPIO));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_ENC_A_GPIO, left_encoder_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_ENC_B_GPIO, left_encoder_isr, NULL));

    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_ENC_A_GPIO, right_encoder_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_ENC_B_GPIO, right_encoder_isr, NULL));
}

static void command_input_init(void)
{
    if (!uart_is_driver_installed(UART_NUM_0)) {
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    }
    uart_input_ready = true;

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    if (!usb_serial_jtag_is_driver_installed()) {
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    }
    usb_input_ready = true;
}

static int read_command_byte(uint8_t *ch)
{
    if (uart_input_ready &&
        uart_read_bytes(UART_NUM_0, ch, 1, 0) == 1) {
        return 1;
    }
    if (usb_input_ready &&
        usb_serial_jtag_read_bytes(ch, 1, 0) == 1) {
        return 1;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return 0;
}

// ===================== 状态显示 =====================

static void print_status(void)
{
    printf("\nStatus:\n");
    printf("  speed_delta_us = %d\n", speed_delta_us);
    printf("  pulse L/R      = %dus / %dus\n", last_left_pulse, last_right_pulse);
    printf("  encoder L/R    = %" PRId64 " / %" PRId64 "\n", left_count, right_count);
    printf("  left reverse   = %d\n", LEFT_MOTOR_REVERSE);
    printf("  right reverse  = %d\n", RIGHT_MOTOR_REVERSE);
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  w = forward\n");
    printf("  s = backward\n");
    printf("  a = turn left\n");
    printf("  d = turn right\n");
    printf("  x = stop\n");
    printf("  1 = left motor forward only\n");
    printf("  2 = left motor backward only\n");
    printf("  3 = right motor forward only\n");
    printf("  4 = right motor backward only\n");
    printf("  c = show encoder count\n");
    printf("  z = show status\n");
    printf("  + = increase speed\n");
    printf("  - = decrease speed\n");
    printf("  h = help\n\n");
}

// ===================== 主程序 =====================

void app_main(void)
{
    // 先初始化输出，让上电尽快进入 1500us 停止状态
    esc_init();
    motor_stop();

    command_input_init();
    encoder_init();

    printf("Holding 1500us neutral for ESC arming (%d ms)...\n",
           ESC_ARM_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_DELAY_MS));

    printf("\n========================================\n");
    printf("ESP32-S3 + APO-DL COMMAND CONTROL\n");
    printf("RC mode: 50Hz, 1000-2000us, 1500us stop\n");
    printf("GPIO4 -> APO-DL A channel S\n");
    printf("GPIO5 -> APO-DL B channel S\n");
    printf("GND   -> APO-DL A/B channel -\n");
    printf("APO-DL A/B channel + not connected\n");
    printf("Keyboard input: UART0 + USB Serial/JTAG\n");
    printf("========================================\n\n");

    printf("Output 1500us stop now.\n");
    printf("If APO-DL was reset, calibrate center at 1500us.\n");

    print_help();
    print_status();
    printf("\n> ");
    fflush(stdout);

    uint8_t ch = 0;

    while (1) {
        int len = read_command_byte(&ch);

        if (len > 0) {
            if (ch == '\r' || ch == '\n') {
                continue;
            }

            printf("\rKEY: %c (0x%02X)\n", ch, (unsigned)ch);

            if (ch == 'w' || ch == 'W') {
                motor_forward();
                printf("CMD: FORWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 's' || ch == 'S') {
                motor_backward();
                printf("CMD: BACKWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'a' || ch == 'A') {
                motor_turn_left();
                printf("CMD: TURN LEFT | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'd' || ch == 'D') {
                motor_turn_right();
                printf("CMD: TURN RIGHT | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'x' || ch == 'X') {
                motor_stop();
                printf("CMD: STOP | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == '1') {
                motor_left_only(1);
                printf("CMD: LEFT ONLY FORWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == '2') {
                motor_left_only(-1);
                printf("CMD: LEFT ONLY BACKWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == '3') {
                motor_right_only(1);
                printf("CMD: RIGHT ONLY FORWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == '4') {
                motor_right_only(-1);
                printf("CMD: RIGHT ONLY BACKWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'c' || ch == 'C') {
                printf("Encoder count L/R: %" PRId64 " / %" PRId64 "\n",
                       left_count, right_count);

            } else if (ch == 'z' || ch == 'Z') {
                print_status();

            } else if (ch == '+') {
                speed_delta_us += SPEED_STEP_US;
                speed_delta_us = clamp_int(speed_delta_us,
                                           MIN_SPEED_DELTA_US,
                                           MAX_SPEED_DELTA_US);
                printf("Speed increased: speed_delta_us = %d\n", speed_delta_us);

            } else if (ch == '-') {
                speed_delta_us -= SPEED_STEP_US;
                speed_delta_us = clamp_int(speed_delta_us,
                                           MIN_SPEED_DELTA_US,
                                           MAX_SPEED_DELTA_US);
                printf("Speed decreased: speed_delta_us = %d\n", speed_delta_us);

            } else if (ch == 'h' || ch == 'H') {
                print_help();

            } else {
                printf("Unknown command: %c\n", ch);
                printf("Press h for help.\n");
            }

            printf("> ");
            fflush(stdout);
        }

    }
}
