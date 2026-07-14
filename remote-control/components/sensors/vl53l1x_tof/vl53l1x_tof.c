#include "vl53l1x_tof.h"

#include <string.h>

#include "VL53L1X_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static vl53l1x_tof_t *s_active_sensor;

vl53l1x_tof_config_t vl53l1x_tof_default_config(void)
{
    vl53l1x_tof_config_t config = {
        .i2c_port = 0,
        .sda_gpio = 39,
        .scl_gpio = 38,
        .scl_speed_hz = 400000,
        .device_address_8bit = VL53L1X_TOF_DEFAULT_ADDR_8BIT,
        .timing_budget_ms = 50,
        .inter_measurement_ms = 55,
        .distance_mode = VL53L1X_TOF_DISTANCE_LONG,
        .external_bus = NULL,
    };
    return config;
}

static esp_err_t ensure_active(uint16_t dev)
{
    if (s_active_sensor == NULL || !s_active_sensor->initialized ||
        dev != s_active_sensor->config.device_address_8bit) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

int8_t VL53L1_WriteMulti(uint16_t dev,
                         uint16_t index,
                         uint8_t *pdata,
                         uint32_t count)
{
    if (ensure_active(dev) != ESP_OK || pdata == NULL) {
        return -1;
    }

    uint8_t buf[258];
    if (count + 2 > sizeof(buf)) {
        return -1;
    }
    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&buf[2], pdata, count);

    return i2c_master_transmit(s_active_sensor->dev,
                               buf,
                               count + 2,
                               100) == ESP_OK
               ? 0
               : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev,
                        uint16_t index,
                        uint8_t *pdata,
                        uint32_t count)
{
    if (ensure_active(dev) != ESP_OK || pdata == NULL) {
        return -1;
    }

    const uint8_t reg[2] = {
        (uint8_t)(index >> 8),
        (uint8_t)(index & 0xFF),
    };
    return i2c_master_transmit_receive(s_active_sensor->dev,
                                       reg,
                                       sizeof(reg),
                                       pdata,
                                       count,
                                       100) == ESP_OK
               ? 0
               : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF),
    };
    return VL53L1_WriteMulti(dev, index, buf, sizeof(buf));
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)(data & 0xFF),
    };
    return VL53L1_WriteMulti(dev, index, buf, sizeof(buf));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    return VL53L1_ReadMulti(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t buf[2] = {0};
    const int8_t status = VL53L1_ReadMulti(dev, index, buf, sizeof(buf));
    if (pdata != NULL) {
        *pdata = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t buf[4] = {0};
    const int8_t status = VL53L1_ReadMulti(dev, index, buf, sizeof(buf));
    if (pdata != NULL) {
        *pdata = ((uint32_t)buf[0] << 24) |
                 ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8) |
                 buf[3];
    }
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return 0;
}

esp_err_t vl53l1x_tof_init(vl53l1x_tof_t *sensor,
                           const vl53l1x_tof_config_t *config)
{
    if (sensor == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;

    if (config->external_bus != NULL) {
        sensor->bus = config->external_bus;
        sensor->owns_bus = false;
    } else {
        const i2c_master_bus_config_t bus_cfg = {
            .i2c_port = config->i2c_port,
            .sda_io_num = config->sda_gpio,
            .scl_io_num = config->scl_gpio,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t ret = i2c_new_master_bus(&bus_cfg, &sensor->bus);
        if (ret != ESP_OK) {
            return ret;
        }
        sensor->owns_bus = true;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address_8bit >> 1,
        .scl_speed_hz = config->scl_speed_hz,
    };
    esp_err_t ret = i2c_master_bus_add_device(sensor->bus, &dev_cfg, &sensor->dev);
    if (ret != ESP_OK) {
        vl53l1x_tof_deinit(sensor);
        return ret;
    }

    sensor->initialized = true;
    s_active_sensor = sensor;

    uint8_t boot_state = 0;
    for (int i = 0; i < 100 && boot_state == 0; ++i) {
        if (VL53L1X_BootState(sensor->config.device_address_8bit, &boot_state) != 0) {
            vl53l1x_tof_deinit(sensor);
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (boot_state == 0) {
        vl53l1x_tof_deinit(sensor);
        return ESP_ERR_TIMEOUT;
    }

    if (VL53L1X_SensorInit(sensor->config.device_address_8bit) != 0 ||
        VL53L1X_SetDistanceMode(sensor->config.device_address_8bit,
                                sensor->config.distance_mode) != 0 ||
        VL53L1X_SetTimingBudgetInMs(sensor->config.device_address_8bit,
                                    sensor->config.timing_budget_ms) != 0 ||
        VL53L1X_SetInterMeasurementInMs(sensor->config.device_address_8bit,
                                        sensor->config.inter_measurement_ms) != 0) {
        vl53l1x_tof_deinit(sensor);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t vl53l1x_tof_read(vl53l1x_tof_t *sensor,
                           vl53l1x_tof_reading_t *out,
                           uint32_t timeout_ms)
{
    if (sensor == NULL || out == NULL || !sensor->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    out->distance_mm = 0;
    out->valid = false;
    s_active_sensor = sensor;

    if (VL53L1X_StartRanging(sensor->config.device_address_8bit) != 0) {
        return ESP_FAIL;
    }

    uint8_t data_ready = 0;
    uint32_t elapsed_ms = 0;
    const uint32_t limit_ms = timeout_ms == 0 ? 250 : timeout_ms;
    while (data_ready == 0 && elapsed_ms < limit_ms) {
        if (VL53L1X_CheckForDataReady(sensor->config.device_address_8bit,
                                      &data_ready) != 0) {
            VL53L1X_StopRanging(sensor->config.device_address_8bit);
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        elapsed_ms += 2;
    }
    if (data_ready == 0) {
        VL53L1X_StopRanging(sensor->config.device_address_8bit);
        return ESP_ERR_TIMEOUT;
    }

    if (VL53L1X_GetDistance(sensor->config.device_address_8bit,
                            &out->distance_mm) != 0 ||
        VL53L1X_ClearInterrupt(sensor->config.device_address_8bit) != 0 ||
        VL53L1X_StopRanging(sensor->config.device_address_8bit) != 0) {
        return ESP_FAIL;
    }

    out->valid = true;
    return ESP_OK;
}

void vl53l1x_tof_deinit(vl53l1x_tof_t *sensor)
{
    if (sensor == NULL) {
        return;
    }
    if (sensor->dev != NULL) {
        i2c_master_bus_rm_device(sensor->dev);
    }
    if (sensor->owns_bus && sensor->bus != NULL) {
        i2c_del_master_bus(sensor->bus);
    }
    if (s_active_sensor == sensor) {
        s_active_sensor = NULL;
    }
    memset(sensor, 0, sizeof(*sensor));
}
