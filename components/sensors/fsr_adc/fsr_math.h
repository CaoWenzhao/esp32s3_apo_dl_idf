#pragma once

typedef struct {
    float slope_v_per_kg;
    float offset_v;
    float min_kg;
    float max_kg;
} fsr_adc_calibration_t;

float fsr_adc_raw_to_voltage(int raw, float reference_voltage_v);
float fsr_adc_voltage_to_weight_kg(float voltage_v,
                                   const fsr_adc_calibration_t *calibration);
