#include "fsr_adc.h"

#include <stddef.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "board_pin_config.h"

static const char *TAG = "fsr_adc";
static fsr_adc_config_t s_config;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_initialized;

#define BATTERY_DIVIDER_TOP_KOHM     98.6f
#define BATTERY_DIVIDER_BOTTOM_KOHM  9.84f
#define BATTERY_6S_EMPTY_V           19.8f
#define BATTERY_6S_FULL_V            25.2f

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

fsr_adc_config_t fsr_adc_default_config(void)
{
    fsr_adc_config_t config = {
        .adc_gpio = PIN_FSR_ADC,
        .adc_channel = FSR_ADC_CHANNEL,
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

esp_err_t fsr_adc_init(const fsr_adc_config_t *config)
{
    if (config == NULL || config->sample_count <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = FSR_ADC_UNIT,
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

    ret = adc_oneshot_config_channel(s_adc_handle,
                                     BATTERY_ADC_CHANNEL,
                                     &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .atten = s_config.atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg,
                                              &s_adc_cali_handle) != ESP_OK) {
        s_adc_cali_handle = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable; using raw conversion");
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ADC FSR GPIO%d/ch%d, battery GPIO%d/ch%d, samples=%d",
             s_config.adc_gpio,
             s_config.adc_channel,
             PIN_BATTERY_ADC,
             BATTERY_ADC_CHANNEL,
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

esp_err_t battery_adc_read(battery_adc_reading_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = (battery_adc_reading_t){0};
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int sum = 0;
    for (int i = 0; i < s_config.sample_count; ++i) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_handle,
                                         BATTERY_ADC_CHANNEL,
                                         &raw);
        if (ret != ESP_OK) {
            return ret;
        }
        sum += raw;
    }

    out->raw = sum / s_config.sample_count;
    int adc_mv = 0;
    if (s_adc_cali_handle == NULL ||
        adc_cali_raw_to_voltage(s_adc_cali_handle, out->raw, &adc_mv) != ESP_OK) {
        adc_mv = (int)((float)out->raw * 3300.0f / 4095.0f + 0.5f);
    }
    out->adc_voltage_v = (float)adc_mv / 1000.0f;
    out->battery_voltage_v = out->adc_voltage_v *
        ((BATTERY_DIVIDER_TOP_KOHM + BATTERY_DIVIDER_BOTTOM_KOHM) /
         BATTERY_DIVIDER_BOTTOM_KOHM);
    out->percent = clampf((out->battery_voltage_v - BATTERY_6S_EMPTY_V) *
                          100.0f / (BATTERY_6S_FULL_V - BATTERY_6S_EMPTY_V),
                          0.0f, 100.0f);
    out->valid = true;
    return ESP_OK;
}

void fsr_adc_deinit(void)
{
    if (s_adc_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(s_adc_cali_handle);
        s_adc_cali_handle = NULL;
    }
    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
    s_initialized = false;
}
