idf_component_register( 
    SRCS
        "bsp.c"

    INCLUDE_DIRS
        "."

    REQUIRES 
        driver
        ms8607
        bno055
        ekf
        bsp
        esp_timer
        tinyusb
)
