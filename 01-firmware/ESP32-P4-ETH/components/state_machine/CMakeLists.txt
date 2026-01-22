idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "."
    REQUIRES freertos shared_memory mqtt_custom_client
)
