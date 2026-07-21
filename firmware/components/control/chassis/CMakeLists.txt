idf_component_register(
    SRCS "chassis.c" "chassis_math.c"
    INCLUDE_DIRS "."
    REQUIRES board esp_driver_ledc esp_driver_gpio
)
