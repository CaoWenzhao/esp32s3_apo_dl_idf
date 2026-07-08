#include "imu_i2c.h"

#include <string.h>

#define IMU_FUNC_VERSION 0x01
#define IMU_FUNC_RAW_ACCEL 0x04
#define IMU_FUNC_RAW_GYRO 0x0A
#define IMU_FUNC_RAW_MAG 0x10
#define IMU_FUNC_QUAT 0x16
#define IMU_FUNC_EULER 0x26
#define IMU_FUNC_BARO 0x32

imu_i2c_config_t imu_i2c_default_config(void)
{
    imu_i2c_config_t config = {
        .i2c_port = 0,
        .sda_gpio = 38,
        .scl_gpio = 37,
        .scl_speed_hz = 400000,
        .device_address = IMU_I2C_DEFAULT_ADDR,
        .external_bus = NULL,
    };
    return config;
}

static int16_t le_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

static float le_float(const uint8_t *bytes)
{
    float value = 0.0f;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

static esp_err_t read_reg(imu_i2c_t *imu, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(imu->dev,
                                       &reg,
                                       1,
                                       buf,
                                       len,
                                       100);
}

esp_err_t imu_i2c_init(imu_i2c_t *imu, const imu_i2c_config_t *config)
{
    if (imu == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(imu, 0, sizeof(*imu));
    imu->config = *config;

    if (config->external_bus != NULL) {
        imu->bus = config->external_bus;
        imu->owns_bus = false;
    } else {
        const i2c_master_bus_config_t bus_cfg = {
            .i2c_port = config->i2c_port,
            .sda_io_num = config->sda_gpio,
            .scl_io_num = config->scl_gpio,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t ret = i2c_new_master_bus(&bus_cfg, &imu->bus);
        if (ret != ESP_OK) {
            return ret;
        }
        imu->owns_bus = true;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->scl_speed_hz,
    };
    esp_err_t ret = i2c_master_bus_add_device(imu->bus, &dev_cfg, &imu->dev);
    if (ret != ESP_OK) {
        if (imu->owns_bus) {
            i2c_del_master_bus(imu->bus);
        }
        memset(imu, 0, sizeof(*imu));
        return ret;
    }

    imu->initialized = true;
    return ESP_OK;
}

void imu_i2c_deinit(imu_i2c_t *imu)
{
    if (imu == NULL || !imu->initialized) {
        return;
    }
    if (imu->dev != NULL) {
        i2c_master_bus_rm_device(imu->dev);
    }
    if (imu->owns_bus && imu->bus != NULL) {
        i2c_del_master_bus(imu->bus);
    }
    memset(imu, 0, sizeof(*imu));
}

esp_err_t imu_i2c_read_version(imu_i2c_t *imu, uint8_t version[3])
{
    if (imu == NULL || version == NULL || !imu->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_reg(imu, IMU_FUNC_VERSION, version, 3);
}

esp_err_t imu_i2c_read_all(imu_i2c_t *imu, imu_i2c_reading_t *out)
{
    if (imu == NULL || out == NULL || !imu->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    uint8_t buf[16] = {0};

    esp_err_t ret = read_reg(imu, IMU_FUNC_RAW_ACCEL, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float accel_ratio = 16.0f / 32767.0f;
    out->accel_g[0] = le_i16(&buf[0]) * accel_ratio;
    out->accel_g[1] = le_i16(&buf[2]) * accel_ratio;
    out->accel_g[2] = le_i16(&buf[4]) * accel_ratio;

    ret = read_reg(imu, IMU_FUNC_RAW_GYRO, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float gyro_ratio = (2000.0f / 32767.0f) * (3.1415926f / 180.0f);
    out->gyro_rad_s[0] = le_i16(&buf[0]) * gyro_ratio;
    out->gyro_rad_s[1] = le_i16(&buf[2]) * gyro_ratio;
    out->gyro_rad_s[2] = le_i16(&buf[4]) * gyro_ratio;

    ret = read_reg(imu, IMU_FUNC_RAW_MAG, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float mag_ratio = 800.0f / 32767.0f;
    out->mag_ut[0] = le_i16(&buf[0]) * mag_ratio;
    out->mag_ut[1] = le_i16(&buf[2]) * mag_ratio;
    out->mag_ut[2] = le_i16(&buf[4]) * mag_ratio;

    ret = read_reg(imu, IMU_FUNC_QUAT, buf, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    out->quat[0] = le_float(&buf[0]);
    out->quat[1] = le_float(&buf[4]);
    out->quat[2] = le_float(&buf[8]);
    out->quat[3] = le_float(&buf[12]);

    ret = read_reg(imu, IMU_FUNC_EULER, buf, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    const float rad_to_deg = 57.2957795f;
    out->euler_deg[0] = le_float(&buf[0]) * rad_to_deg;
    out->euler_deg[1] = le_float(&buf[4]) * rad_to_deg;
    out->euler_deg[2] = le_float(&buf[8]) * rad_to_deg;

    ret = read_reg(imu, IMU_FUNC_BARO, buf, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    out->baro[0] = le_float(&buf[0]);
    out->baro[1] = le_float(&buf[4]);
    out->baro[2] = le_float(&buf[8]);
    out->baro[3] = le_float(&buf[12]);
    out->valid = true;
    return ESP_OK;
}
