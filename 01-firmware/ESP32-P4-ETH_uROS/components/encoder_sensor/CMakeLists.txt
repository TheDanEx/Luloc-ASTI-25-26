idf_component_register(
    SRCS "encoder_sensor.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer driver
)
