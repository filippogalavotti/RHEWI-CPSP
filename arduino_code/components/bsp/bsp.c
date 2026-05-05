#include "bsp.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "bsp";

esp_err_t bsp_i2c_init(i2c_master_bus_handle_t *pt_bus_handle, i2c_master_bus_handle_t *imu_bus_handle) 
{
    esp_err_t ret;
    i2c_master_bus_config_t pt_bus_config = 
    {
        .i2c_port = MS8607_MASTER_NUM,
        .sda_io_num = MS8607_SDA,
        .scl_io_num = MS8607_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if((ret = i2c_new_master_bus(&pt_bus_config, pt_bus_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed MS8607 I2C bus init: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to ensure bus is ready before initializing the second one

    i2c_master_bus_config_t imu_bus_config = 
    {
        .i2c_port = BNO055_MASTER_NUM,
        .sda_io_num = BNO055_SDA,
        .scl_io_num = BNO055_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if((ret = i2c_new_master_bus(&imu_bus_config, imu_bus_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed BNO055 I2C bus init: %s", esp_err_to_name(ret));
        return ret;
    };
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "I2C initialized");

    return ESP_OK;
}

esp_err_t bsp_gpio_init(void) 
{
    esp_err_t ret;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_RED) | (1ULL << LED_GREEN) | (1ULL << LED_BLUE),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    if((ret = gpio_config(&io_conf)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed leds init: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_GREEN, 1);
    gpio_set_level(LED_BLUE, 1);

    ESP_LOGI(TAG, "Leds initialized");

    return ESP_OK;
}