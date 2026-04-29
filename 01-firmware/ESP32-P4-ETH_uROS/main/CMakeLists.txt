idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        shared_memory
        esp_timer
        freertos
        log
)