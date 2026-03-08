idf_component_register(
    SRCS "ina219_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer driver log
)
