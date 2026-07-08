idf_component_register(SRCS "a02yyuw.c" "sw_uart.c" INCLUDE_DIRS "." REQUIRES 
esp_driver_gptimer log
esp_driver_uart esp_driver_gpio esp_timer)
