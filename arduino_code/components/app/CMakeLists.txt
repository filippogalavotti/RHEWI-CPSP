idf_component_register(
    SRCS
        "app_context.c"
        "app_hil.c"
        "app_sensors.c"
        "control_task.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        esp_timer
        ms8607
        bno055
        ekf
        bsp
        tinyusb
)
