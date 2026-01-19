idf_component_register(
    SRCS "ethernet.c"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_netif esp_event driver log
)