#ifndef ETHERNET_H
#define ETHERNET_H

#include "esp_err.h"

// Configuración de IP Estática deseada
#define STATIC_IP_ADDR  "192.168.1.100"
#define STATIC_NETMASK  "255.255.255.0"
#define STATIC_GW_ADDR  "192.168.1.1"

// Inicializa Ethernet con IP fija
void ethernet_init_with_static_ip(void);

#endif // ETHERNET_H