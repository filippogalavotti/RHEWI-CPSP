#include "control_task.h"
#include "app_context.h"
#include "app_hil.h"
#include "app_sensors.h"
#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern app_context_t g_app;
extern sensors_t g_sensors;

/* --- Blink task function -------------------------------------------------------------------------- */
static void vBlinkTask(void *pvParameters)
{
#if APP_MODE == APP_MODE_HIL
    app_context_t *ctx = &g_app;
#endif

    for (;;) {       
#if APP_MODE == APP_MODE_HIL
        if(ctx->running) {
            gpio_set_level(LED_RED,   0); gpio_set_level(LED_GREEN, 1); gpio_set_level(LED_BLUE,  1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_RED,   1); gpio_set_level(LED_GREEN, 1); gpio_set_level(LED_BLUE,  1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            gpio_set_level(LED_RED,   1); gpio_set_level(LED_GREEN, 0); gpio_set_level(LED_BLUE,  1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_RED,   1); gpio_set_level(LED_GREEN, 1); gpio_set_level(LED_BLUE,  1);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
#else
            gpio_set_level(LED_RED,   1); gpio_set_level(LED_GREEN, 1); gpio_set_level(LED_BLUE,  0);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_RED,   1); gpio_set_level(LED_GREEN, 1); gpio_set_level(LED_BLUE,  1);
            vTaskDelay(pdMS_TO_TICKS(200));
#endif
    }
}

/* --- Control task function ------------------------------------------------ */
static void vControlTask(void* pvParameters) 
{    
    app_context_t *ctx = &g_app;
#if APP_MODE == APP_MODE_HIL      
    app_hil_loop(ctx);
#else
    sensors_t *sns = &g_sensors;
    app_sensors_loop(ctx, sns);
#endif
}

/* --- Start application tasks --------------------------------------------- */
void start_app(void)
{
    xTaskCreate(vControlTask, "control", 16384, NULL, 6, NULL);
    xTaskCreate(vBlinkTask, "blink", 1024, NULL, 1, NULL);
}