idf_component_register(
    SRCS "curvature_feedforward.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt_custom_client esp_timer log
)
