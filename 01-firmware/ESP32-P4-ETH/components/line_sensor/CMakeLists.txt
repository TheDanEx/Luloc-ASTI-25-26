idf_component_register(
    SRCS "line_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer esp_adc freertos
)
