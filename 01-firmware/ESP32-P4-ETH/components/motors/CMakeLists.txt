idf_component_register(
    SRCS "motor.c" "encoder_read.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer driver
)
