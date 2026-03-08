idf_component_register(
    SRCS "mqtt_api_responder.c"
    INCLUDE_DIRS "include"
    REQUIRES log json mqtt_custom_client shared_memory test_sensor 
)
