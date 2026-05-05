#pragma once
#include "esp_log.h"
#include "app_context.h"

/**
 * @brief Initialize the sensors by setting up the I2C communication and configuring the sensors for data acquisition.
 * @return ESP_OK if initialization is successful, otherwise an error code.
 */
esp_err_t app_sensors_init(void);

/**
 * @brief Handle the sensor data acquisition and processing in a loop, reading data from the sensors at a specified frequency and updating the global sensor structure with the latest readings.
 * @param sensors Pointer to the global sensors structure where the latest sensor readings will be stored.
 */
void app_sensors_loop(app_context_t* ctx, sensors_t *sensors);