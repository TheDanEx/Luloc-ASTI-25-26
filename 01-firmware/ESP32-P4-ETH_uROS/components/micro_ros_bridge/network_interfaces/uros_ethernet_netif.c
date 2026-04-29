#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif
#include "uros_network_interfaces.h"

#ifdef CONFIG_MICRO_ROS_ESP_NETIF_ENET

static const char *TAG = "eth_interface";

static EventGroupHandle_t s_eth_event_group;
#define ETH_CONNECTED_BIT BIT0
#define ETH_GOT_IP_BIT   BIT1

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Mask:   " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "GW:     " IPSTR, IP2STR(&ip_info->gw));

    xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
}

esp_err_t uros_network_interface_initialize(void)
{
    s_eth_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Hardware reset the PHY
    if (CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO),
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io_conf);
        gpio_set_level(CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_MICRO_ROS_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO;

#if CONFIG_MICRO_ROS_USE_INTERNAL_ETHERNET
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = CONFIG_MICRO_ROS_ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = CONFIG_MICRO_ROS_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
#if CONFIG_MICRO_ROS_ETH_PHY_IP101
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_RTL8201
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_LAN8720
    esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_DP83848
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_KSZ8041
    esp_eth_phy_t *phy = esp_eth_phy_new_ksz8041(&phy_config);
#endif
#elif CONFIG_ETH_USE_SPI_ETHERNET
    gpio_install_isr_service(0);
    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_MICRO_ROS_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_MICRO_ROS_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_MICRO_ROS_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_MICRO_ROS_ETH_SPI_HOST, &buscfg, 1));
#if CONFIG_MICRO_ROS_USE_DM9051
    spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .mode = 0,
        .clock_speed_hz = CONFIG_MICRO_ROS_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_MICRO_ROS_ETH_SPI_CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg, &spi_handle));
    eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(spi_handle);
    dm9051_config.int_gpio_num = CONFIG_MICRO_ROS_ETH_SPI_INT_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_dm9051(&phy_config);
#elif CONFIG_MICRO_ROS_USE_W5500
    spi_device_interface_config_t devcfg = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = CONFIG_MICRO_ROS_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_MICRO_ROS_ETH_SPI_CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg, &spi_handle));
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
    w5500_config.int_gpio_num = CONFIG_MICRO_ROS_ETH_SPI_INT_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
#endif
#endif // CONFIG_ETH_USE_SPI_ETHERNET

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

#if CONFIG_ETH_USE_SPI_ETHERNET
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    }));
#endif

    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

#ifdef CONFIG_MICRO_ROS_ETH_STATIC_IP
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));
    esp_netif_ip_info_t ip_info = {0};
    esp_netif_str_to_ip4(CONFIG_MICRO_ROS_ETH_STATIC_IP_ADDR, &ip_info.ip);
    esp_netif_str_to_ip4(CONFIG_MICRO_ROS_ETH_STATIC_NETMASK, &ip_info.netmask);
    esp_netif_str_to_ip4(CONFIG_MICRO_ROS_ETH_STATIC_GW, &ip_info.gw);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));
    ESP_LOGI(TAG, "Static IP: " IPSTR, IP2STR(&ip_info.ip));
#endif

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    // Block until Ethernet link is up (max 10s)
    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group,
        ETH_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    if (!(bits & ETH_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Ethernet link did not come up within 10s");
    }

#ifndef CONFIG_MICRO_ROS_ETH_STATIC_IP
    bits = xEventGroupWaitBits(s_eth_event_group,
        ETH_GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & ETH_GOT_IP_BIT)) {
        ESP_LOGW(TAG, "Did not obtain IP via DHCP within 15s");
    }
#endif

    return ESP_OK;
}

#endif