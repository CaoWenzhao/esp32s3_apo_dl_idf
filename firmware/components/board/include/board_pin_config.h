#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/adc_types.h"

/* Algorithm 6 fixed board wiring. Keep all application code on these names. */
#define PIN_ESC_LEFT_PWM              GPIO_NUM_4
#define PIN_ESC_RIGHT_PWM             GPIO_NUM_5

#define PIN_ENCODER_LEFT_A            GPIO_NUM_6
#define PIN_ENCODER_LEFT_B            GPIO_NUM_7
#define PIN_ENCODER_RIGHT_A           GPIO_NUM_15
#define PIN_ENCODER_RIGHT_B           GPIO_NUM_16

#define RPLIDAR_UART_PORT             UART_NUM_2
#define PIN_RPLIDAR_TX                GPIO_NUM_17
#define PIN_RPLIDAR_RX                GPIO_NUM_18

#define UWB_UART_PORT                 UART_NUM_1
#define PIN_UWB_TX                    GPIO_NUM_47
#define PIN_UWB_RX                    GPIO_NUM_48

#define PIN_ULTRASONIC_LEFT_RX        GPIO_NUM_38
#define PIN_ULTRASONIC_RIGHT_RX       GPIO_NUM_39

#define PIN_FSR_ADC                   GPIO_NUM_8
#define FSR_ADC_UNIT                  ADC_UNIT_1
#define FSR_ADC_CHANNEL               ADC_CHANNEL_7

#define PIN_BATTERY_ADC               GPIO_NUM_3
#define BATTERY_ADC_UNIT              ADC_UNIT_1
#define BATTERY_ADC_CHANNEL           ADC_CHANNEL_2

#define PIN_IMU_SCL                   GPIO_NUM_41
#define PIN_IMU_SDA                   GPIO_NUM_42

#define UWB_BAUD_RATE                 115200
#define RPLIDAR_BAUD_RATE             460800
#define ULTRASONIC_BAUD_RATE          9600

#define ESC_PWM_FREQUENCY_HZ          50
#define ESC_PWM_PERIOD_US             20000
#define ESC_PWM_MIN_US                1000
#define ESC_PWM_NEUTRAL_US            1500
#define ESC_PWM_MAX_US                2000
#define ESC_PWM_RESOLUTION_BITS       14
#define ESC_ARM_TIME_MS               2000
