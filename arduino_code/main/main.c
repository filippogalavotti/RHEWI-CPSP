#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp.h"
#include "app_context.h"
#include "app_hil.h"
#include "app_sensors.h"
#include "control_task.h"

/* Main */
void app_main(void)
{
    bsp_gpio_init();

#if APP_MODE == APP_MODE_HIL
        app_hil_init();
#else
        app_sensors_init();
#endif

    start_app();

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY));
    }       
}