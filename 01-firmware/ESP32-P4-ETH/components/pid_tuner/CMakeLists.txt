idf_component_register(
    SRCS "pid_tuner.c"
    INCLUDE_DIRS "include"
    REQUIRES log nvs_flash json mqtt_custom_client shared_memory 
)
