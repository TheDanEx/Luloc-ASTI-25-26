idf_component_register(SRCS "ethernet.c"
                       REQUIRES esp_eth
                       PRIV_REQUIRES esp_driver_gpio esp_netif esp_event
                       INCLUDE_DIRS ".")
