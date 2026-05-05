#include "ms8607.h"
#include "bsp.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "math.h"

/**
 * @brief Helper: Apply the compensation calculations to the raw temperature and pressure values read from the MS8607 sensor using the calibration values, and store the compensated values in the sensor structure.
 * @param sensor Pointer to the ms8607_sensor_t structure containing the raw values and calibration
 */
void ms8607_ptsensor_compensation(ms8607_sensor_t* sensor)
{
    uint16_t SENST1 = sensor->calib_coeffs[0];
    uint16_t OFFT1 = sensor->calib_coeffs[1];
    uint16_t TCS = sensor->calib_coeffs[2];
    uint16_t TCO = sensor->calib_coeffs[3];
    uint16_t TREF = sensor->calib_coeffs[4];
    uint16_t TEMPSENS = sensor->calib_coeffs[5];

    /* Temperature compensation */
    int32_t dT = (int32_t)sensor->latest_sample.raw_temperature - ((int32_t)TREF << 8);
    int32_t TEMP = 2000 + (((int64_t)dT *(int64_t)TEMPSENS) >> 23); // Actual temperature

    /* First order pressure compensation */
    int64_t OFF = ((int64_t)OFFT1 << 17) + (((int64_t)TCO * (int64_t)dT) >> 6);
    int64_t SENS = ((int64_t)SENST1 << 16) + (((int64_t)TCS * (int64_t)dT) >> 7);

    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    /* Second order temperature compensation */
    if(TEMP < 2000) {
        T2 = 3*((int64_t)dT*(int64_t)dT) >> 33;
        OFF2 = 61*((int64_t)TEMP-2000)*((int64_t)TEMP-2000) >> 4;
        SENS2 = 29*((int64_t)TEMP-2000)*((int64_t)TEMP-2000) >> 4;
        if(TEMP < -1500) {
            OFF2 += 17*((int64_t)TEMP+1500)*((int64_t)TEMP+1500) >> 4;
            SENS2 += 9*((int64_t)TEMP+1500)*((int64_t)TEMP+1500) >> 4;
        }
    }
    else {
        T2 = 5*((int64_t)dT*(int64_t)dT) >> 38;
        OFF2 = 0;
        SENS2 = 0;
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = (((sensor->latest_sample.raw_pressure * SENS) >> 21) - OFF) >> 15; // Actual pressure
    sensor->latest_sample.temperature_c = (float)TEMP/100;
    sensor->latest_sample.pressure_mbar = (float)P/100;
}

/**
 * @brief Helper: Check the CRC of the calibration values fetched from the MS8607 sensor.
 * @param n_prom Pointer to the array of calibration values.
 * @return true if the CRC check passes, false otherwise.
 */
bool ms8607_crc_check(uint16_t *n_prom) 
{
    uint8_t crc = (n_prom[0] & 0xF000) >> 12;
    uint8_t cnt, n_bit;
    uint16_t n_rem, crc_read;

    n_rem = 0x00;
    crc_read = n_prom[0];
    n_prom[7] = 0;
    n_prom[0] = (0x0FFF & (n_prom[0])); // Clear the CRC byte

    for (cnt = 0; cnt < (7 + 1) * 2; cnt++) {
    // Get next byte
        if (cnt % 2 == 1) 
            n_rem ^= n_prom[cnt >> 1] & 0x00FF;
        else
            n_rem ^= n_prom[cnt >> 1] >> 8;

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000)
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem <<= 1;
        }
    }   
    n_rem = ((n_rem >> 12) & 0x000F); // Final 4-bit reminder is CRC code
    n_prom[0] = crc_read;
    return (n_rem == crc);
}

/**
 * @brief Helper: Read the raw ADC values for temperature and pressure from the MS8607 sensor by sending the appropriate conversion commands and reading the results.
 * @param sensor Pointer to the ms8607_sensor_t structure where the raw values will be stored.
 * @return ESP_OK if the raw values are successfully read, otherwise an error code.
 */
esp_err_t ms8607_read_raw_pt_values(i2c_master_dev_handle_t handle, ms8607_sample_t *sample)
{
    esp_err_t ret;
    uint8_t command; 
    uint8_t rx_buffer[3];

    /* Start temperature conversion*/
    command = CONVERT_D2_OSR_4096; // Using a resolution of 4096 for temperature measurement
    if((ret = i2c_master_transmit(handle, &command, 1, 100)) != ESP_OK){
        ESP_LOGE("MS8607", "Failed to start temperature ADC conversion: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(9));

    /* Read temperature */
    command = ADC_READ;
    if((ret = i2c_master_transmit_receive(handle, &command, 1, rx_buffer, sizeof(rx_buffer),100)) != ESP_OK){
        ESP_LOGE("MS8607", "Failed to start temperature ADC conversion: %s", esp_err_to_name(ret));
        return ret;
    }
    uint32_t raw_temperature = ((uint32_t)rx_buffer[0] << 16 | (uint32_t)rx_buffer[1] << 8 | (uint32_t)rx_buffer[2]);
    sample->raw_temperature = raw_temperature;

    /* Start pressure conversion*/
    command = CONVERT_D1_OSR_4096; // Using a resolution of 4096 for pressure measurement
    if((ret = i2c_master_transmit(handle, &command, 1, 100)) != ESP_OK){
        ESP_LOGE("MS8607", "Failed to start pressure ADC conversion: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(9));

    /* Read pressure */
    command = ADC_READ;
    if((ret = i2c_master_transmit_receive(handle, &command, 1, rx_buffer, sizeof(rx_buffer),100)) != ESP_OK){
        ESP_LOGE("MS8607", "Failed to start pressure ADC conversion: %s", esp_err_to_name(ret));
        return ret;
    }
    uint32_t raw_pressure = ((uint32_t)rx_buffer[0] << 16 | (uint32_t)rx_buffer[1] << 8 | (uint32_t)rx_buffer[2]);
    sample->raw_pressure = raw_pressure;

    sample->timestamp_us = (uint64_t)esp_timer_get_time();

    return ESP_OK;
}

/* User API */

esp_err_t ms8607_init(ms8607_sensor_t *sensor, i2c_master_bus_handle_t pt_bus)
{
    esp_err_t ret;

    i2c_device_config_t pt_config = 
    {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = MS8607_PT_ADDRESS,
        .scl_speed_hz       = MS8607_MASTER_FREQ_kHZ * 1000,
    };
    if((ret = i2c_master_bus_add_device(pt_bus, &pt_config, &sensor->pt_handle)) != ESP_OK) {
        ESP_LOGE("MS8607", "Failed to add pressure and temperature device: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t hum_config = 
    {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = MS8607_HUM_ADDRESS,
        .scl_speed_hz       = MS8607_MASTER_FREQ_kHZ * 1000,
    };
    if((ret = i2c_master_bus_add_device(pt_bus, &hum_config, &sensor->hum_handle)) != ESP_OK) {
        ESP_LOGE("MS8607", "Failed to add humidity device: %s", esp_err_to_name(ret));
        return ret;
    }

    if((ret = ms8607_reset(*sensor)) != ESP_OK) {
        ESP_LOGE("MS8607", "Failed to reset sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(15));

    if((ret = ms8607_fetch_calibration_values(sensor)) != ESP_OK) {
        ESP_LOGE("MS8607", "Failed to fetch calibration values: %s", esp_err_to_name(ret));
        return ret;
    }

    float p0 = 0;
    for(int i=0; i<100; i++){
        ESP_ERROR_CHECK(ms8607_get_pt_values(sensor));
        p0 += sensor->latest_sample.pressure_mbar;
        vTaskDelay(pdMS_TO_TICKS(50));   // 50ms → ~5s
    }
    sensor->p0 = p0/100; // Average pressure at startup as reference pressure for altitude calculation

    ESP_LOGI("MS8607", "Device found");

    return ESP_OK;
}

esp_err_t ms8607_reset(ms8607_sensor_t sensor)
{
    esp_err_t ret;
    uint8_t reset_command = HSENSOR_RESET_COMMAND;
    if(((ret = i2c_master_transmit(sensor.hum_handle, &reset_command, 1, 100)) != ESP_OK)) {
        ESP_LOGE("MS8607", "Failed to reset humidity sensor command: %s", esp_err_to_name(ret));
        return ret;
    }
    reset_command = P_T_RESET;
    if(((ret = i2c_master_transmit(sensor.pt_handle, &reset_command, 1, 100)) != ESP_OK)) {
        ESP_LOGE("MS8607", "Failed to reset pressure and temperature sensor command: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t ms8607_fetch_calibration_values(ms8607_sensor_t* sensor)
{
    uint8_t offset = 0;
    uint16_t buffer[8];
    uint8_t rx_buffer[2];
    esp_err_t ret;

    for(int i = 0; i < 7; i++) {
        offset = 2 * i;
        uint8_t prom_read_cmd = PROM_ADDRESS_READ_ADDRESS_0 + offset;
        if((ret = i2c_master_transmit_receive(sensor->pt_handle, &prom_read_cmd, 1, rx_buffer, 2, 100)) != ESP_OK) {
            ESP_LOGE("MS8607", "Failed to read calibration value %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
        buffer[i] = (rx_buffer[0] << 8) | rx_buffer[1];
    }

    if(!ms8607_crc_check(buffer)) {
        ESP_LOGE("MS8607", "Calibration data CRC check failed");
        return ESP_ERR_INVALID_CRC;
    }

    sensor->calib_coeffs[0] = buffer[1];
    sensor->calib_coeffs[1] = buffer[2];
    sensor->calib_coeffs[2] = buffer[3];
    sensor->calib_coeffs[3] = buffer[4];
    sensor->calib_coeffs[4] = buffer[5];
    sensor->calib_coeffs[5] = buffer[6];

    return ESP_OK;
}

esp_err_t ms8607_get_pt_values(ms8607_sensor_t* sensor)
{
    if(ms8607_read_raw_pt_values(sensor->pt_handle, &sensor->latest_sample) == ESP_OK) {
        ms8607_ptsensor_compensation(sensor);
    }
    else {
        ESP_LOGE("MS8607", "Failed to read raw pressure and temperature values");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

float ms8607_compute_altitude(ms8607_sensor_t sensor)
{
    // Using the barometric formula to compute altitude from pressure
    // altitude = 44330 * (1 - (P / P0)^(1/5.255))
    float ratio = sensor.latest_sample.pressure_mbar / sensor.p0;
    float raw_altitude = 44330.0f * (1.0f - powf(ratio, 0.1903f));

    return raw_altitude;

}