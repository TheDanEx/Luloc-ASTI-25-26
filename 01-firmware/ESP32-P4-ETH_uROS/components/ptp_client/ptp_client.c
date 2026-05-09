/*
 * PTP (IEEE 1588) & SNTP Client Implementation for ESP32-P4
 * SPDX-License-Identifier: MIT
 */

#include "ptp_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Networking headers for UDP/Multicast
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PTP_MULTICAST_IP "224.0.1.129"
#define PTP_EVENT_PORT   319
#define PTP_GENERAL_PORT 320

static const char *TAG = "ptp_client";

// Global internal state
static volatile int64_t master_slave_offset_us = 0;
static volatile bool is_synchronized = false;
static TaskHandle_t ptp_task_handle = NULL;

static void ptp_listener_task(void *arg)
{
    ESP_LOGI(TAG, "PTP Listener Task started on Core %d", xPortGetCoreID());

    // 1. Setup UDP Socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create IPv4 socket");
        vTaskDelete(NULL);
        return;
    }

    // 2. Bind to INADDR_ANY, listening mostly to PTP General Port (Follow_Up)
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PTP_GENERAL_PORT), // Bind to General Port (320)
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    // 3. Join Multicast Group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(PTP_MULTICAST_IP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGW(TAG, "Failed to join IGMP %s. UDP PTP might fail.", PTP_MULTICAST_IP);
    }

    // 4. Timeout for non-blocking loop
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t rx_buffer[128];

    while (1) {
        // Fallback SNTP
        if (!is_synchronized) {
            struct timeval sntp_tv;
            gettimeofday(&sntp_tv, NULL);
            if (sntp_tv.tv_sec > 1704067200LL) {
                uint64_t sntp_epoch_us = ((uint64_t)sntp_tv.tv_sec * 1000000ULL) + sntp_tv.tv_usec;
                master_slave_offset_us = sntp_epoch_us - esp_timer_get_time();
            }
        }

        int len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
        if (len >= 44) {
             uint8_t msg_id = rx_buffer[0] & 0x0F;
             
             // Sync (0x0) or Follow_Up (0x8)
             if (msg_id == 0x0 || msg_id == 0x8) { 
                 uint64_t sec = 0;
                 for(int i = 0; i < 6; i++) {
                     sec = (sec << 8) | rx_buffer[34 + i];
                 }
                 uint32_t nsec = 0;
                 for(int i = 0; i < 4; i++) {
                     nsec = (nsec << 8) | rx_buffer[40 + i];
                 }
                 
                 // If valid PTP Timestamp is contained
                 if (sec > 1000000000ULL) {
                     uint64_t master_epoch_us = (sec * 1000000ULL) + (nsec / 1000UL);
                     int64_t local_up_us = esp_timer_get_time();
                     
                     master_slave_offset_us = master_epoch_us - local_up_us;
                     
                     if (!is_synchronized) {
                         ESP_LOGI(TAG, "PTP Locked: %lld us offset", master_slave_offset_us);
                         is_synchronized = true;
                     }
                 }
             }
        }
    }
}

esp_err_t ptp_client_init(void)
{
    if (ptp_task_handle != NULL) return ESP_OK;

    // Start SNTP Fallback against the fixed Raspberry IP (No Internet needed)
    ESP_LOGI(TAG, "Initializing SNTP Fallback (192.168.5.1)");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "192.168.5.1");
    esp_sntp_init();

    BaseType_t res = xTaskCreatePinnedToCore(
        ptp_listener_task, "ptp_rx", 4096, NULL, 5, &ptp_task_handle, 1
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed creating PTP task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

uint64_t get_ptp_timestamp_us(void)
{
    int64_t local_up_time = esp_timer_get_time();
    int64_t master_time = local_up_time + master_slave_offset_us;
    if (master_time < 0) return 0;
    return (uint64_t)master_time;
}

bool ptp_client_is_synced(void)
{
    return is_synchronized;
}
