idf_component_register(
    SRCS
        "vl53l1x_tof.c"
        "VL53L1X_api.c"
        "VL53L1X_calibration.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
