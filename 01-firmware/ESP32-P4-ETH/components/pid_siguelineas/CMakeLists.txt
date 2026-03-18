idf_component_register(
    SRCS "pid_siguelineas.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer log
)
