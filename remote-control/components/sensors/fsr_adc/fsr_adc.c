#include "fsr_adc.h"

#include <stddef.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "fsr_adc";
static fsr_adc_config_t s_config;
static adc_oneshot_unit_handle_t s_adc_handle;
static bool s_initialized;

fsr_adc_config_t fsr_adc_default_config(void)
{
    fsr_adc_config_t config = {
        .adc_gpio = 8,
        .adc_channel = 7,
        .atten = ADC_ATTEN_DB_12,
        .sample_count = 10,
        .reference_voltage_v = 3.3f,
        .calibration = {
            .slope_v_per_kg = 0.0004f,
            .offset_v = 0.0749f,
            .min_kg = 0.0f,
            .max_kg = 6.0f,
        },
    };
    return config;
}

float fsr_adc_raw_to_voltage(int raw, float reference_voltage_v)
{
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 4095) {
        raw = 4095;
    }
    if (reference_voltage_v <= 0.0f) {
        reference_voltage_v = 3.3f;
    }
    return ((float)raw * reference_voltage_v) / 4095.0f;
}

float fsr_adc_voltage_to_weight_kg(float voltage_v,
                                   const fsr_adc_calibration_t *calibration)
{
    if (calibration == NULL || calibration->slope_v_per_kg <= 0.0f) {
        return 0.0f;
    }

    float weight_kg = (voltage_v - calibration->offset_v) /
                      calibration->slope_v_per_kg;
    if (weight_kg < calibration->min_kg) {
        weight_kg = calibration->min_kg;
    }
    if (weight_kg > calibration->max_kg) {
        weight_kg = calibration->max_kg;
    }
    return weight_kg;
}

esp_err_t fsr_adc_init(const fsr_adc_config_t *config)
{
    if (config == NULL || config->sample_count <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = s_config.atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(s_adc_handle,
                                     s_config.adc_channel,
                                     &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ADC GPIO%d channel=%d samples=%d",
             s_config.adc_gpio,
             s_config.adc_channel,
             s_config.sample_count);
    return ESP_OK;
}

esp_err_t fsr_adc_read(fsr_adc_reading_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    out->raw = 0;
    out->voltage_v = 0.0f;
    out->weight_kg = 0.0f;
    out->valid = false;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int sum = 0;
    for (int i = 0; i < s_config.sample_count; ++i) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_handle, s_config.adc_channel, &raw);
        if (ret != ESP_OK) {
            return ret;
        }
        sum += raw;
    }

    out->raw = sum / s_config.sample_count;
    out->voltage_v = fsr_adc_raw_to_voltage(out->raw, s_config.reference_voltage_v);
    out->weight_kg = fsr_adc_voltage_to_weight_kg(out->voltage_v,
                                                  &s_config.calibration);
    out->valid = true;
    return ESP_OK;
}

void fsr_adc_deinit(void)
{
    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
    s_initialized = false;
}
