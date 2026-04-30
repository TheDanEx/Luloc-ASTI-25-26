idf_component_register(
    SRCS "ina226_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer driver log audio_player
)
