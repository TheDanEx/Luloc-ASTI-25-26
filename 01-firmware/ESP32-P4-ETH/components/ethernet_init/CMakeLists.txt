idf_component_register(SRCS "ethernet_init.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth esp_netif esp_event
                       INCLUDE_DIRS ".")
