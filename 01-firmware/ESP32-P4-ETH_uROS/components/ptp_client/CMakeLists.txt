idf_component_register(
    SRCS "ptp_client.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer freertos lwip esp_netif
)
