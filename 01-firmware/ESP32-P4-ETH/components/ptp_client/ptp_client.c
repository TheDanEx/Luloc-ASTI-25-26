/*
 * PTP (IEEE 1588) & SNTP Client Implementation for ESP32-P4
 * SPDX-License-Identifier: MIT
 */

#include "ptp_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_custom_client.h"
#include <stdlib.h>
#include <math.h>
#include "esp_eth_mac_esp.h"
#include "ethernet.h"

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
static int64_t last_sync_offset_us = 0;
static TaskHandle_t ptp_task_handle = NULL;

// PI Controller for Hardware Clock steering
static double s_adj_scale_factor = 1.0;
static int64_t s_integral_error_us = 0;
static const double Kp = 0.1;  // Proportional gain
static const double Ki = 0.01; // Integral gain
static const double MAX_ADJ = 0.001; // Max 1000ppm adjustment

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
                     
                     // Get Hardware RX Timestamp for this packet
                     uint16_t seq_id = (rx_buffer[30] << 8) | rx_buffer[31];
                     eth_mac_time_t hw_rx_ts;
                     int64_t local_rx_us;
                     
                     if (ethernet_get_ptp_rx_timestamp(seq_id, &hw_rx_ts) == ESP_OK) {
                         local_rx_us = (int64_t)hw_rx_ts.seconds * 1000000LL + (hw_rx_ts.nanoseconds / 1000LL);
                         ESP_LOGD(TAG, "Hardware TS Match! Seq: %u", seq_id);
                     } else {
                         // Fallback to software if HW TS not found (should be rare)
                         local_rx_us = get_ptp_timestamp_us();
                         ESP_LOGW(TAG, "Hardware TS NOT found for Seq: %u, using SW fallback", seq_id);
                     }
                     
                     int64_t current_offset_us = (int64_t)master_epoch_us - local_rx_us;
                     
                     // PI Controller logic
                     if (!is_synchronized || llabs(current_offset_us) > 1000000LL) {
                         // First sync or very large jump: Hard set the clock
                         eth_mac_time_t hard_ts = {
                             .seconds = (uint32_t)(master_epoch_us / 1000000ULL),
                             .nanoseconds = (uint32_t)((master_epoch_us % 1000000ULL) * 1000ULL)
                         };
                         esp_eth_ioctl(ethernet_get_handle(), ETH_MAC_ESP_CMD_S_PTP_TIME, &hard_ts);
                         s_integral_error_us = 0;
                         s_adj_scale_factor = 1.0;
                         ESP_LOGI(TAG, "PTP Initial HW Clock Set: %llu s", (uint64_t)hard_ts.seconds);
                     } else {
                         // Fine adjustment (Frequency Steering)
                         s_integral_error_us += current_offset_us;
                         
                         // Limit integral to prevent windup
                         if (s_integral_error_us > 1000000) s_integral_error_us = 1000000;
                         if (s_integral_error_us < -1000000) s_integral_error_us = -1000000;
                         
                         double adj = (Kp * (double)current_offset_us + Ki * (double)s_integral_error_us) / 1000000.0;
                         
                         // Clamp adjustment
                         if (adj > MAX_ADJ) adj = MAX_ADJ;
                         if (adj < -MAX_ADJ) adj = -MAX_ADJ;
                         
                         s_adj_scale_factor = 1.0 + adj;
                         esp_eth_ioctl(ethernet_get_handle(), ETH_MAC_ESP_CMD_ADJ_PTP_FREQ, &s_adj_scale_factor);
                     }

                     int64_t delta_us = (last_sync_offset_us == 0) ? 0 : (current_offset_us - last_sync_offset_us);
                     last_sync_offset_us = current_offset_us;
                     
                     ESP_LOGI(TAG, "PTP HW Update | Error: %lld us | Jitter: %lld us | FreqAdj: %.6f", 
                              current_offset_us, delta_us, s_adj_scale_factor);

                     if (!is_synchronized) {
                             master_slave_offset_us = current_offset_us;
                             
                            // Calcular la hora local ajustada de Espana (CET/CEST)
                            setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
                            tzset();
                            
                            time_t locked_time_sec = (time_t)(master_epoch_us / 1000000ULL);
                            struct tm timeinfo;
                            localtime_r(&locked_time_sec, &timeinfo);
                            
                            char strftime_buf[64];
                            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
                            
                            ESP_LOGI(TAG, "PTP Date/Time (Spain): %s", strftime_buf);


                             is_synchronized = true;

                             // Set Spain timezone implicitly for the C STDLIB
                             setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
                             tzset();
                             
                             char time_str[64];
                             strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

#ifdef CONFIG_TELEMETRY_ROBOT_NAME
                             const char *robot_name = CONFIG_TELEMETRY_ROBOT_NAME;
#else
                             const char *robot_name = "unknown";
#endif
                             // Note: InfluxDB ILP timestamps should be in nanoseconds
                             char ilp[256];
                             snprintf(ilp, sizeof(ilp), "events,type=TIME_SYNC,robot=%s sync_date=\"%s\" %lld", 
                                     robot_name, time_str, master_epoch_us * 1000ULL);
                             
                             mqtt_custom_client_publish("robot/events", ilp, 0, 1, 0);
                         }               }
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
    eth_mac_time_t hw_ts;
    if (esp_eth_ioctl(ethernet_get_handle(), ETH_MAC_ESP_CMD_G_PTP_TIME, &hw_ts) == ESP_OK) {
        return (uint64_t)hw_ts.seconds * 1000000ULL + (hw_ts.nanoseconds / 1000ULL);
    }
    // Fallback if Ethernet not ready
    return (uint64_t)esp_timer_get_time() + master_slave_offset_us;
}

bool ptp_client_is_synced(void)
{
    return is_synchronized;
}

void ptp_client_force_sync(void)
{
    is_synchronized = false;
    esp_sntp_restart();
}
