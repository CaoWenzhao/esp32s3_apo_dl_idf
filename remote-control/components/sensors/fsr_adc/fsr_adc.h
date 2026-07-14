#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/adc_types.h"

typedef struct {
    float slope_v_per_kg;
    float offset_v;
    float min_kg;
    float max_kg;
} fsr_adc_calibration_t;

typedef struct {
    int adc_gpio;
    adc_channel_t adc_channel;
    adc_atten_t atten;
    int sample_count;
    float reference_voltage_v;
    fsr_adc_calibration_t calibration;
} fsr_adc_config_t;

typedef struct {
    int raw;
    float voltage_v;
    float weight_kg;
    bool valid;
} fsr_adc_reading_t;

fsr_adc_config_t fsr_adc_default_config(void);
float fsr_adc_raw_to_voltage(int raw, float reference_voltage_v);
float fsr_adc_voltage_to_weight_kg(float voltage_v,
                                   const fsr_adc_calibration_t *calibration);
esp_err_t fsr_adc_init(const fsr_adc_config_t *config);
esp_err_t fsr_adc_read(fsr_adc_reading_t *out);
void fsr_adc_deinit(void);

