idf_component_register(
    SRCS "pid_tuner.c"
    INCLUDE_DIRS "include"
    REQUIRES log nvs_flash shared_memory
)
