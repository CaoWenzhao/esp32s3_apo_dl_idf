#include "fsr_math.h"

#include <stddef.h>

float fsr_adc_raw_to_voltage(int raw, float reference_voltage_v)
{
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    if (reference_voltage_v <= 0.0f) reference_voltage_v = 3.3f;
    return ((float)raw * reference_voltage_v) / 4095.0f;
}

float fsr_adc_voltage_to_weight_kg(float voltage_v,
                                   const fsr_adc_calibration_t *calibration)
{
    if (calibration == NULL || calibration->slope_v_per_kg <= 0.0f) return 0.0f;
    float weight = (voltage_v - calibration->offset_v) /
                   calibration->slope_v_per_kg;
    if (weight < calibration->min_kg) weight = calibration->min_kg;
    if (weight > calibration->max_kg) weight = calibration->max_kg;
    return weight;
}
