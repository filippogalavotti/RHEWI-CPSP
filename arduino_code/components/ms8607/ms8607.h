#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>

#define MS8607_PT_ADDRESS 0x76 // The pressure and temperature I2C address for the sensor
#define MS8607_HUM_ADDRESS 0x40 // The default relative humidity I2C address for the sensor

/* Humidity sensor device commands*/
#define HSENSOR_RESET_COMMAND 0xFE
#define HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND 0xE5 
#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND 0xF5
#define HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND 0xFA0F
#define HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND 0xFCC9
#define HSENSOR_WRITE_USER_REG_COMMAND 0xE6
#define HSENSOR_READ_USER_REG_COMMAND 0xE7 

/* Processing constants */
#define HSENSOR_TEMPERATURE_COEFFICIENT (float)(-0.15)
#define HSENSOR_CONSTANT_A (float)(8.1332)
#define HSENSOR_CONSTANT_B (float)(1762.39)
#define HSENSOR_CONSTANT_C (float)(235.66)

/* Coefficients for temperature computation */
#define TEMPERATURE_COEFF_MUL (175.72) 
#define TEMPERATURE_COEFF_ADD (-46.85) 

/* Coefficients for relative humidity computation */
#define HUMIDITY_COEFF_MUL (125) 
#define HUMIDITY_COEFF_ADD (-6) 

/* Humidity sensor conversion timings */
#define HSENSOR_CONVERSION_TIME_12b 16
#define HSENSOR_CONVERSION_TIME_10b 5 
#define HSENSOR_CONVERSION_TIME_8b 3  
#define HSENSOR_CONVERSION_TIME_11b 9 

/* Humidity sensor user register masks and bit position */
#define HSENSOR_USER_REG_RESOLUTION_MASK 0x81 
#define HSENSOR_USER_REG_END_OF_BATTERY_MASK 0x40 
#define HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK 0x4 
#define HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK 0x2 

#define MS8607_RH_ADDRESS (0x40) // Humidity I2C address for the sensor
#define MS8607_RH_LSB 0.0019073486328125; // value for each count coming from the humidity register
// PSENSOR commands
#define PROM_ADDRESS_READ_ADDRESS_0 0xA0

/* Pressure & Temperature commands */
#define P_T_RESET 0x1E           
#define CONVERT_D1_OSR_256 0x40  
#define CONVERT_D1_OSR_512 0x42  
#define CONVERT_D1_OSR_1024 0x44 
#define CONVERT_D1_OSR_2048 0x46 
#define CONVERT_D1_OSR_4096 0x48 
#define CONVERT_D1_OSR_8192 0x4A 
#define CONVERT_D2_OSR_256 0x50  
#define CONVERT_D2_OSR_512 0x52  
#define CONVERT_D2_OSR_1024 0x54 
#define CONVERT_D2_OSR_2048 0x56 
#define CONVERT_D2_OSR_4096 0x58 
#define CONVERT_D2_OSR_8192 0x5A 
#define ADC_READ 0x00 

/* Commands for relative humidity */
#define HUM_RESET 0xFE
#define HUM_WRITE_REGISTER 0xE6
#define HUM_READ_REGISTER 0xE7
#define HUM_MEASURE_RH_HOLD 0xE5
#define HUM_MEASURE_RH_NO_HOLD 0xF5 

/* Commands for pressure and temperature */
#define PSENSOR_RESET_COMMAND 0x1E
#define PSENSOR_START_PRESSURE_ADC_CONVERSION 0x40
#define PSENSOR_START_TEMPERATURE_ADC_CONVERSION 0x50 
#define PSENSOR_READ_ADC 0x00 

typedef struct {
    uint32_t raw_pressure;  // Raw pressure value read from the sensor
    uint32_t raw_temperature;   // Raw temperature value read from the sensor

    float temperature_c;    // Compensated temperature in degrees Celsius
    float pressure_mbar;    // Compensated pressure in millibars
    float humidity_rh;  // Compensated relative humidity in percentage

    uint64_t timestamp_us;  // Timestamp of the sample in microseconds since the sensor was read
} ms8607_sample_t;

typedef struct {
    i2c_master_dev_handle_t pt_handle;  // I2C handle for pressure and temperature sensor
    i2c_master_dev_handle_t hum_handle; // I2C handle for humidity sensor

    uint16_t calib_coeffs[6];   // Calibration coefficients read from the sensor's PROM, used for compensation calculations
    float p0; // baseline pressure for altitude estimation
    ms8607_sample_t latest_sample;  // Latest compensated sample containing temperature, pressure, humidity, and timestamp
    float altitude_m; // computed altitude based on pressure and p0
} ms8607_sensor_t;

/**
 * @brief Initialize the MS8607 sensor by setting up the I2C communication and resetting the sensor.
 * @param sensor Pointer to the ms8607_sensor_t structure where the sensor handles and calibration values will be stored.
 * @param pt_bus The I2C bus handle to which the sensor is connected.
 * @return ESP_OK if initialization is successful, otherwise an error code.
 */
esp_err_t ms8607_init(ms8607_sensor_t* sensor, i2c_master_bus_handle_t pt_bus);

/**
 * @brief Reset the MS8607 sensor by sending the appropriate reset commands to both the pressure/temperature and humidity sensors.
 * @param sensor The ms8607_sensor_t structure containing the sensor handles.
 * @return ESP_OK if the reset commands are successfully sent, otherwise an error code.
 */
esp_err_t ms8607_reset(ms8607_sensor_t sensor);

/**
 * @brief Fetch the calibration values from the MS8607 sensor's PROM and store them in a buffer for later use in compensation calculations.
 * @param sensor Pointer to the ms8607_sensor_t structure where the calibration values will be stored.
 * @return ESP_OK if the calibration values are successfully fetched, otherwise an error code.
 */
esp_err_t ms8607_fetch_calibration_values(ms8607_sensor_t* sensor);

/**
 * @brief Get the compensated temperature and pressure values from the MS8607 sensor by reading the raw ADC values, applying compensation calculations, and logging the results.
 * @param sensor Pointer to the ms8607_sensor_t structure where the compensated values will be stored.
 * @return ESP_OK if the values are successfully read and compensated, otherwise an error code.
 */
esp_err_t ms8607_get_pt_values(ms8607_sensor_t* sensor);

/**
 * @brief Compute the altitude based on the current pressure reading and the baseline pressure.
 * @param sensor The ms8607_sensor_t structure containing the sensor data.
 * @return The computed altitude in meters.
 */
float ms8607_compute_altitude(ms8607_sensor_t sensor);