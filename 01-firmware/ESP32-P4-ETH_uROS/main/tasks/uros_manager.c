#include "uros_manager.h"
#include "shared_memory.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "UROS_MGR";

#define STRING_BUFFER_LEN 160
#define SYNC_INTERVAL_MS 5000
#define WHEEL_BASE_M 0.170f

#define RCCHECK(fn) do { \
    rcl_ret_t temp_rc = (fn); \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int)temp_rc); \
        return; \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t temp_rc = (fn); \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing...", __LINE__, (int)temp_rc); \
    } \
} while (0)

static rcl_publisher_t diag_publisher;
static rcl_publisher_t my_publisher;
// static rcl_subscription_t my_subscriber;
static rcl_subscription_t velocity_subscriber;
static rcl_subscription_t mode_subscriber;

static std_msgs__msg__String diag_msg;
static std_msgs__msg__Int32 send_msg;
static std_msgs__msg__Int8 change_mode;
// static std_msgs__msg__Int32 recv_msg;
static geometry_msgs__msg__Twist cmd_vel;

static float g_latency_ms = 0.0f;
static float g_offset_ms = 0.0f;
static float g_jitter_ms = 0.0f;
static float g_last_latency_ms = -1.0f;

// static void subscription_callback(const void *msvin)
// {
//     const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msvin;
//     ESP_LOGI(TAG, "Received from topic: %ld", (long)msg->data);
// }

static void mode_callback(const void *msvin)
{
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msvin;
    printf("Mode change -> %d \n",msg->data);
}

static uint32_t millis_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void subscription_vel_callback(const void *msvin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msvin;

    float v = msg->linear.x;
    float w = msg->angular.z;
    float target_l = v - (w*WHEEL_BASE_M/2.0);
    float target_r = v + (w*WHEEL_BASE_M/2.0);

    uint32_t now = millis_now();

    shared_memory_t* shm = shared_memory_get();
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        shm->teleop.target_speed_left = target_l;
        shm->teleop.target_speed_right = target_r;
        shm->teleop.last_update_ms = now;
        xSemaphoreGive(shm->mutex);
    }  
    printf("cmd_vel -> lin.x: %.2f ang.z: %.2f\n",(float)msg->linear.x,(float)msg->angular.z);
    
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);

    char strftime_buf[32];
    strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);

    snprintf(diag_msg.data.data,
             diag_msg.data.capacity,
             "[SYNC] %s.%06ld | Lat: %.3fms | Off: %.3fms | Jit: %.3fms",
             strftime_buf,
             (long)tv.tv_usec,
             g_latency_ms,
             g_offset_ms,
             g_jitter_ms);

    diag_msg.data.size = strlen(diag_msg.data.data);

    RCSOFTCHECK(rcl_publish(&diag_publisher, &diag_msg, NULL));

    send_msg.data++;
    RCSOFTCHECK(rcl_publish(&my_publisher, &send_msg, NULL));
}

static void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro_ros_task started");
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();

    while (1) {
        rclc_support_t support;
        rcl_node_t node;
        rcl_timer_t timer;
        rclc_executor_t executor;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options =
            rcl_init_options_get_rmw_init_options(&init_options);

        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rmw_options
        );
#endif

        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        node = rcl_get_zero_initialized_node();
        RCCHECK(rclc_node_init_default(&node, "esp32_p4_robot", "", &support));

        RCCHECK(rclc_publisher_init_default(
            &diag_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/microROS/esp_diag_time"
        ));

        RCCHECK(rclc_publisher_init_default(
            &my_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/esp32/publisher"
        ));

        // RCCHECK(rclc_subscription_init_default(
        //     &my_subscriber,
        //     &node,
        //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        //     "/esp32/subscriber"
        // ));

        RCCHECK(rclc_subscription_init_default(
            &velocity_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"
        ));

        RCCHECK(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(2000),
            timer_callback,
            true
        ));

        RCCHECK(rclc_subscription_init_default(
            &mode_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "/robot/mode_cmd"
        ));
        
        RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        // RCCHECK(rclc_executor_add_subscription(
        //     &executor,
        //     &my_subscriber,
        //     &recv_msg,
        //     &subscription_callback,
        //     ON_NEW_DATA
        // ));

        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &velocity_subscriber,
            &cmd_vel,
            &subscription_vel_callback,
            ON_NEW_DATA
        ));
        
        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &mode_subscriber,
            &change_mode,
            &mode_callback,
            ON_NEW_DATA
        ));
        static char buffer[STRING_BUFFER_LEN];
        diag_msg.data.data = buffer;
        diag_msg.data.capacity = STRING_BUFFER_LEN;
        diag_msg.data.size = 0;

        send_msg.data = 0;

        int64_t last_sync_time = 0;

        while (1) {
            int64_t now = esp_timer_get_time();

            if ((now - last_sync_time) > (SYNC_INTERVAL_MS * 1000)) {
                int64_t t_before = esp_timer_get_time();

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
                if (rmw_uros_sync_session(1000) == RCL_RET_OK) {
                    int64_t t_after = esp_timer_get_time();
                    int64_t rtt_us = t_after - t_before;

                    g_latency_ms = (float)rtt_us / 2000.0f;

                    if (g_last_latency_ms >= 0) {
                        g_jitter_ms = fabsf(g_latency_ms - g_last_latency_ms);
                    }

                    g_last_latency_ms = g_latency_ms;

                    int64_t agent_ms = rmw_uros_epoch_millis();
                    int64_t agent_us = (agent_ms * 1000) + (rtt_us / 2);

                    struct timeval tv_now;
                    gettimeofday(&tv_now, NULL);

                    int64_t local_us =
                        (int64_t)tv_now.tv_sec * 1000000 + tv_now.tv_usec;

                    g_offset_ms = (float)(agent_us - local_us) / 1000.0f;

                    struct timeval tv_sync = {
                        .tv_sec = agent_us / 1000000,
                        .tv_usec = agent_us % 1000000
                    };

                    settimeofday(&tv_sync, NULL);
                    last_sync_time = esp_timer_get_time();
                }
#endif
            }

            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        rcl_publisher_fini(&diag_publisher, &node);
        rcl_publisher_fini(&my_publisher, &node);
        // rcl_subscription_fini(&my_subscriber, &node);
        rcl_subscription_fini(&velocity_subscriber, &node);
        rcl_subscription_fini(&mode_subscriber, &node);
        rcl_timer_fini(&timer);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        rcl_init_options_fini(&init_options);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t uros_manager_start(void)
{
    static bool started = false;
    ESP_LOGI(TAG,"Inicio micro-ROS");
    if (started) {
        ESP_LOGW(TAG, "micro-ROS task already started");
        return ESP_OK;
    }

    BaseType_t result = xTaskCreate(
        micro_ros_task,
        "uros_task",
        16384,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS task");
        return ESP_FAIL;
    }

    started = true;
    return ESP_OK;
}