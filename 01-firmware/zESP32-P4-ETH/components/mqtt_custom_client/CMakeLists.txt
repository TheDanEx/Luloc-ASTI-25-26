idf_component_register(
    SRCS "mqtt_custom_client.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES mqtt esp_event esp_netif nvs_flash
)
