#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "ethernet.h"

static const char *TAG = "ETH_HAL";

// Pines definidos según el esquemático del Waveshare ESP32-P4-ETH [1]
#define ETH_PHY_ADDR        -1           // Dirección por defecto del IP101
#define ETH_PHY_RST_GPIO    -1          // Reset gestionado por hardware/RC en esta placa
#define ETH_MDC_GPIO        22
#define ETH_MDIO_GPIO       23

// Pines de datos RMII (ESP32-P4)
#define ETH_RMII_CLK_GPIO   50          // Entrada de reloj 50MHz desde el PHY
#define ETH_RMII_TX_EN      49
#define ETH_RMII_TX0        41
#define ETH_RMII_TX1        42
#define ETH_RMII_RX0        52
#define ETH_RMII_RX1        53
#define ETH_RMII_CRS_DV     51

/** Manejador de eventos (Solo para log, el Ping lo responde LwIP automáticamente) */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_CONNECTED) {
        esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
        uint8_t mac_addr[2] = {0};
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up - MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr, mac_addr[3], mac_addr[4], mac_addr[5], mac_addr[6], mac_addr[7]);
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP Asignada: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void ethernet_init_with_static_ip(void)
{
    // 1. Inicializar la red TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. Crear la interfaz de red por defecto para Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // --- CONFIGURACIÓN DE IP FIJA ---
    // Detenemos el cliente DHCP antes de asignar IP estática
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));

    esp_netif_ip_info_t ip_info;
    esp_netif_str_to_ip4(STATIC_IP_ADDR, &ip_info.ip);
    esp_netif_str_to_ip4(STATIC_NETMASK, &ip_info.netmask);
    esp_netif_str_to_ip4(STATIC_GW_ADDR, &ip_info.gw);
    
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));
    // --------------------------------

    // 3. Configuración del MAC (Media Access Controller) del ESP32-P4
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = ETH_MDIO_GPIO;
    
    // Configuración del reloj RMII: IMPORTANTE para Waveshare ESP32-P4-ETH
    // El PHY genera los 50MHz y entran al GPIO 50 [1][8]
    esp32_emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN; 
    esp32_emac_config.clock_config.rmii.clock_gpio = ETH_RMII_CLK_GPIO;

    // Mapeo de pines de datos [9][1]
    esp32_emac_config.emac_dataif_gpio.rmii.tx_en_num = ETH_RMII_TX_EN;
    esp32_emac_config.emac_dataif_gpio.rmii.txd0_num = ETH_RMII_TX0;
    esp32_emac_config.emac_dataif_gpio.rmii.txd1_num = ETH_RMII_TX1;
    esp32_emac_config.emac_dataif_gpio.rmii.rxd0_num = ETH_RMII_RX0;
    esp32_emac_config.emac_dataif_gpio.rmii.rxd1_num = ETH_RMII_RX1;
    esp32_emac_config.emac_dataif_gpio.rmii.crs_dv_num = ETH_RMII_CRS_DV;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    // 4. Configuración del PHY (IP101 para esta placa)
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR; // Dirección 1 habitualmente
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO; 
    
    // Usamos el driver específico para IP101 incluido en IDF
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    // 5. Instalar y adjuntar el driver
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    // Unir el driver Ethernet con la pila TCP/IP (LwIP)
    void *glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));

    // 6. Registrar eventos y arrancar
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(TAG, "Ethernet iniciado con IP estática");
}