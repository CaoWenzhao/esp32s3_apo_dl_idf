idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        a02yyuw
        board
        bu_uwb
        fsr_adc
        rplidar_c1
        chassis
        follow_avoid
        ble_control
        web_control
        esp_timer
        esp_driver_ledc
        esp_driver_gpio
        esp_driver_i2c
        nvs_flash
)
