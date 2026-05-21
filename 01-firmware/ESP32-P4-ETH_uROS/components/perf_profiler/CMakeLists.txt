idf_component_register(
    SRCS "perf_profiler.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer mqtt_custom_client cjson log
)
