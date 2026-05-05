#include "bno055.h"
#include "bsp.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

static const char *TAG = "BNO055";

/**
 * @brief Helper: Read data from a specified register of the BNO055 sensor.
 * @param reg_address The address of the register to read from.
 * @param data A pointer to a buffer where the read data will be stored.
 * @param data_len The length of the data to read in bytes.
 * @return ESP_OK if the read is successful, otherwise an error code.
 */
static esp_err_t bno055_read_register(i2c_master_dev_handle_t handle, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    return i2c_master_transmit_receive(handle, &reg_address, 1, data, data_len, 100);
}

/**
 * @brief Helper: Write data to a specified register of the BNO055 sensor.
 * @param reg_address The address of the register to write to.
 * @param data A pointer to the data to be written.
 * @param data_len The length of the data to write in bytes.
 * @return ESP_OK if the write is successful, otherwise an error code.
 */
static esp_err_t bno055_write8(i2c_master_dev_handle_t handle, uint8_t reg_address, uint8_t data)
{
    uint8_t write_buffer[2];
    write_buffer[0] = reg_address;
    write_buffer[1] = data;
    esp_err_t ret = i2c_master_transmit(handle, write_buffer, 2, 100);

    return ret;
}

/**
 * @brief Helper: Convert raw accelerometer and gyroscope data from the BNO055 sensor to SI units (m/s^2 for acceleration and rad/s for angular velocity).
 * @param s A pointer to a bno055_sample_t structure containing the raw data to be converted and where the converted values will be stored.
 */
static void bno055_convert(bno055_sample_t *s)
{
    const float ACC_LSB_TO_MPS2 = 1.0f / 100.0f;       // 0.01 m/s^2 per LSB
    const float GYR_LSB_TO_DPS  = 1.0f / 16.0f;        // 0.0625 dps per LSB
    const float DEG_TO_RAD      = 0.017453292519943295f;
    const float QUAT_LSB        = 1.0f / 16384.0f;   // quaternion scale

    for (int i = 0; i < 3; ++i) {
        s->acc_mps2[i]  = (float)s->acc_raw[i]  * ACC_LSB_TO_MPS2;
        float gyro_dps  = (float)s->gyro_raw[i] * GYR_LSB_TO_DPS;
        s->gyro_radps[i]= gyro_dps * DEG_TO_RAD;
    }

    for(int i=0;i<4;i++)
        s->quat[i] = s->quat_raw[i] * QUAT_LSB;
}

/* User API */

esp_err_t bno055_reset(i2c_master_dev_handle_t handle)
{
    esp_err_t ret;
    if((ret = bno055_write8(handle, BNO055_SYS_TRIGGER_ADDR, 0x00)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset");
        return ret;
    };
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t bno055_set_mode(i2c_master_dev_handle_t handle, uint8_t op_mode)
{
    esp_err_t ret;
    if((ret = bno055_write8(handle, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode: %d", op_mode);
        return ret;
    };
    vTaskDelay(pdMS_TO_TICKS(30));
    if((ret = bno055_write8(handle, BNO055_OPR_MODE_ADDR, op_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode: %d", op_mode);
        return ret;
    };
    vTaskDelay(pdMS_TO_TICKS(30));
    return ESP_OK;
}

esp_err_t bno055_set_power_mode(i2c_master_dev_handle_t handle, uint8_t power_mode)
{
    esp_err_t ret;
    if((ret = bno055_write8(handle, BNO055_PWR_MODE_ADDR, power_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power mode: %d", power_mode);
        return ret;
    };
    vTaskDelay(pdMS_TO_TICKS(30));
    return ESP_OK;
}

esp_err_t bno055_get_agq_values(i2c_master_dev_handle_t handle, bno055_sample_t *out, bno055_status_t *status)
{
    uint8_t buf[8];
    esp_err_t ret;

    if((ret = bno055_calibration_status(handle, status)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration status");
        return ret;
    }

    uint8_t reg = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
    if((ret = bno055_read_register(handle, reg, buf, 6)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }
    out->acc_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    out->acc_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    out->acc_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    reg = BNO055_GYRO_DATA_X_LSB_ADDR;
    if((ret = bno055_read_register(handle, reg, buf, 6)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }
    out->gyro_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    out->gyro_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    out->gyro_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    if((ret = bno055_read_register(handle, reg, buf, 8)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read quaternion data");
        return ret;
    }
    out->quat_raw[0] = (int16_t)((buf[1] << 8) | buf[0]); // W
    out->quat_raw[1] = (int16_t)((buf[3] << 8) | buf[2]); // X
    out->quat_raw[2] = (int16_t)((buf[5] << 8) | buf[4]); // Y
    out->quat_raw[3] = (int16_t)((buf[7] << 8) | buf[6]); // Z


    out->timestamp_us = (uint64_t)esp_timer_get_time();

    bno055_convert(out);
    return ESP_OK;
}

esp_err_t bno055_axis_remap(i2c_master_dev_handle_t handle, uint8_t axis_map_config, uint8_t axis_map_sign)
{
    // Devi essere in CONFIGMODE prima di chiamarla.
    esp_err_t ret;
    if((ret = bno055_write8(handle, BNO055_AXIS_MAP_CONFIG_ADDR, axis_map_config)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set axis map config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if((ret = bno055_write8(handle, BNO055_AXIS_MAP_SIGN_ADDR, axis_map_sign)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set axis map sign");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t bno055_calibration_status(i2c_master_dev_handle_t handle, bno055_status_t *s)
{
    esp_err_t ret;
    uint8_t buffer;

    if((ret = bno055_read_register(handle, BNO055_CALIB_STAT_ADDR, &buffer, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration status");
        return ret;
    }
    s->sys_calib = (buffer >> 6) & 0x03;
    s->gyr_calib = (buffer >> 4) & 0x03;
    s->acc_calib = (buffer >> 2) & 0x03;
    s->mag_calib = buffer & 0x03;

    return ESP_OK;
}

esp_err_t bno055_init(bno055_sensor_t *sensor, i2c_master_bus_handle_t bus)
{
    esp_err_t ret;
    uint8_t id;

    i2c_device_config_t dev_config = 
    {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = BNO055_ADDR,
        .scl_speed_hz       = BNO055_MASTER_FREQ_kHZ * 1000,
    };
    if((ret = i2c_master_bus_add_device(bus, &dev_config, &sensor->imu_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(bno055_read_register(sensor->imu_handle, BNO055_CHIP_ID_ADDR, &id, 1));
    if (id != 0xA0) {
        ESP_LOGE(TAG, "Device not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Device found");
    
    ESP_ERROR_CHECK(bno055_set_mode(sensor->imu_handle, OPERATION_MODE_CONFIG));

    ESP_ERROR_CHECK(bno055_reset(sensor->imu_handle));

    ESP_ERROR_CHECK(bno055_set_power_mode(sensor->imu_handle, POWER_MODE_NORMAL));
    
    ESP_ERROR_CHECK(bno055_write8(sensor->imu_handle, BNO055_PAGE_ID_ADDR, 0));

    ESP_ERROR_CHECK(bno055_set_mode(sensor->imu_handle, OPERATION_MODE_IMUPLUS));

    return ESP_OK;
}