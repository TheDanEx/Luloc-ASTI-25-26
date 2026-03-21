idf_component_register(
    SRCS "telemetry_manager.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt_custom_client ptp_client
)
