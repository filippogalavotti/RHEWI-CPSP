#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* BNO055 I2C configuration */
#define BNO055_MASTER_NUM I2C_NUM_1
#define BNO055_SDA GPIO_NUM_2
#define BNO055_SCL GPIO_NUM_3
#define BNO055_MASTER_FREQ_kHZ 300

/* MS8607 I2C configuration */
#define MS8607_MASTER_NUM I2C_NUM_0
#define MS8607_SCL GPIO_NUM_11
#define MS8607_SDA GPIO_NUM_12
#define MS8607_MASTER_FREQ_kHZ 300

/* LED configuration */
#define LED_RED GPIO_NUM_46
#define LED_GREEN GPIO_NUM_0
#define LED_BLUE GPIO_NUM_45

/**
 * @brief Initialize I2C buses
 * @param pt_bus_handle Pointer to store the handle of the MS8607 I2C bus
 * @param imu_bus_handle Pointer to store the handle of the BNO055 I2C bus
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t bsp_i2c_init(i2c_master_bus_handle_t *pt_bus_handle, i2c_master_bus_handle_t *imu_bus_handle);

/**
 * @brief Initialize GPIOs for LEDs
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t bsp_gpio_init(void);