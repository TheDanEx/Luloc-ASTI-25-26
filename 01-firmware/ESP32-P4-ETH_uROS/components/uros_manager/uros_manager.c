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
#include "esp_system.h"
#include "esp_timer.h"

// micro-ROS headers
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>

// Messages
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// =============================================================================
// Constants & Config
// =============================================================================

static const char *TAG = "UROS_MGR";
#define SYNC_INTERVAL_MS 5000
#define FAST_TELEMETRY_INTERVAL_MS 100  // 10Hz
#define SLOW_TELEMETRY_INTERVAL_MS 1000 // 1Hz
#define EXECUTOR_HANDLES 10

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing...",__LINE__,(int)temp_rc);}}

// =============================================================================
// Static Variables (Private)
// =============================================================================

static rcl_publisher_t fast_telemetry_pub;
static rcl_publisher_t slow_telemetry_pub;
static rcl_publisher_t log_pub;

static rcl_subscription_t mode_sub;
static rcl_subscription_t twist_sub;

static std_msgs__msg__Float32MultiArray fast_telemetry_msg;
static std_msgs__msg__Float32MultiArray slow_telemetry_msg;
static std_msgs__msg__String log_msg;

static std_msgs__msg__Int8 recv_mode_msg;
static geometry_msgs__msg__Twist recv_twist_msg;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer_fast;
static rcl_timer_t timer_slow;

static float g_latency_ms = 0.0f;
static float g_jitter_ms = 0.0f;
static float g_last_latency_ms = -1.0f;

// =============================================================================
// Memory Management for micro-ROS
// =============================================================================

static void * microros_allocate(size_t size, void * state) {
    return pvPortMalloc(size);
}

static void microros_deallocate(void * pointer, void * state) {
    vPortFree(pointer);
}

static void * microros_reallocate(void * pointer, size_t size, void * state) {
    return realloc(pointer, size);
}

static void * microros_zero_allocate(size_t n, size_t size, void * state) {
    return calloc(n, size);
}

// =============================================================================
// Callbacks
// =============================================================================

void mode_callback(const void * msin) {
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msin;
    robot_mode_cmd_t cmd = { .new_mode = msg->data };
    shared_memory_put_mode_cmd(&cmd);
}

void twist_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
    robot_twist_cmd_t cmd = { .linear_x = msg->linear.x, .angular_z = msg->angular.z };
    shared_memory_put_twist_cmd(&cmd);
}

void timer_fast_callback(rcl_timer_t * timer, int64_t last_call_time) {
    robot_telemetry_fast_t data;
    if (shared_memory_get_telemetry_fast(&data) == ESP_OK) {
        for(int i=0; i<8; i++) fast_telemetry_msg.data.data[i] = data.line_raw[i];
        for(int i=0; i<8; i++) fast_telemetry_msg.data.data[i+8] = data.line_norm[i];
        fast_telemetry_msg.data.data[16] = data.pid_error;
        fast_telemetry_msg.data.data[17] = data.pid_setpoint;
        fast_telemetry_msg.data.data[18] = data.pid_output;
        fast_telemetry_msg.data.data[19] = data.pid_p;
        fast_telemetry_msg.data.data[20] = data.pid_i;
        fast_telemetry_msg.data.data[21] = data.pid_d;
        
        RCSOFTCHECK(rcl_publish(&fast_telemetry_pub, &fast_telemetry_msg, NULL));
    }
}

void timer_slow_callback(rcl_timer_t * timer, int64_t last_call_time) {
    robot_telemetry_slow_t data;
    if (shared_memory_get_telemetry_slow(&data) == ESP_OK) {
        slow_telemetry_msg.data.data[0] = data.odom_x;
        slow_telemetry_msg.data.data[1] = data.odom_y;
        slow_telemetry_msg.data.data[2] = data.odom_theta;
        slow_telemetry_msg.data.data[3] = data.odom_lin;
        slow_telemetry_msg.data.data[4] = data.odom_ang;
        for(int i=0; i<8; i++) slow_telemetry_msg.data.data[i+5] = data.calib_min[i];
        for(int i=0; i<8; i++) slow_telemetry_msg.data.data[i+13] = data.calib_max[i];
        slow_telemetry_msg.data.data[21] = data.battery_v;
        slow_telemetry_msg.data.data[22] = (float)data.current_mode;
        slow_telemetry_msg.data.data[23] = (float)data.uptime_s;
        slow_telemetry_msg.data.data[24] = g_latency_ms;
        slow_telemetry_msg.data.data[25] = g_jitter_ms;

        RCSOFTCHECK(rcl_publish(&slow_telemetry_pub, &slow_telemetry_msg, NULL));
    }
}

// =============================================================================
// Task Implementation
// =============================================================================

static void task_uros_core0(void * arg) {
    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    allocator.state = NULL;

    if (!rcutils_set_default_allocator(&allocator)) {
        ESP_LOGE(TAG, "Failed to set default allocator");
        vTaskDelete(NULL);
    }

    while(1) {
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABE, rmw_options));
#endif

        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));
        vTaskDelay(pdMS_TO_TICKS(100));

        // Publishers (Reduced to avoid middleware limits)
        RCCHECK(rclc_publisher_init_default(&fast_telemetry_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/telemetry_fast"));
        vTaskDelay(pdMS_TO_TICKS(50));
        RCCHECK(rclc_publisher_init_default(&slow_telemetry_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/telemetry_slow"));
        vTaskDelay(pdMS_TO_TICKS(50));
        RCCHECK(rclc_publisher_init_default(&log_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot/logs"));
        vTaskDelay(pdMS_TO_TICKS(50));

        // Subscribers
        RCCHECK(rclc_subscription_init_default(&mode_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/robot/mode_cmd"));
        RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

        // Timers
        RCCHECK(rclc_timer_init_default2(&timer_fast, &support, RCL_MS_TO_NS(FAST_TELEMETRY_INTERVAL_MS), timer_fast_callback, true));
        RCCHECK(rclc_timer_init_default2(&timer_slow, &support, RCL_MS_TO_NS(SLOW_TELEMETRY_INTERVAL_MS), timer_slow_callback, true));

        // Executor
        RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator));
        RCCHECK(rclc_executor_add_subscription(&executor, &mode_sub, &recv_mode_msg, &mode_callback, ON_NEW_DATA));
        RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &recv_twist_msg, &twist_callback, ON_NEW_DATA));
        RCCHECK(rclc_executor_add_timer(&executor, &timer_fast));
        RCCHECK(rclc_executor_add_timer(&executor, &timer_slow));

        // Msg Buffers
        fast_telemetry_msg.data.data = (float*)pvPortMalloc(22 * sizeof(float)); fast_telemetry_msg.data.capacity = fast_telemetry_msg.data.size = 22;
        slow_telemetry_msg.data.data = (float*)pvPortMalloc(26 * sizeof(float)); slow_telemetry_msg.data.capacity = slow_telemetry_msg.data.size = 26;
        log_msg.data.data = (char*)pvPortMalloc(256); log_msg.data.capacity = 256; log_msg.data.size = 0;

        int64_t last_sync_time = 0;

        while(1) {
            int64_t now = esp_timer_get_time();
            if ((now - last_sync_time) > (SYNC_INTERVAL_MS * 1000)) {
                last_sync_time = now;
                if (rmw_uros_sync_session(1000) == RCL_RET_OK) {
                    int64_t t_before = esp_timer_get_time();
                    int64_t agent_ms = rmw_uros_epoch_millis();
                    g_latency_ms = (float)(esp_timer_get_time() - t_before) / 2000.0f;
                    if (g_last_latency_ms >= 0) g_jitter_ms = fabsf(g_latency_ms - g_last_latency_ms);
                    g_last_latency_ms = g_latency_ms;

                    struct timeval tv_sync = { .tv_sec = agent_ms / 1000, .tv_usec = (agent_ms % 1000) * 1000 };
                    settimeofday(&tv_sync, NULL);
                } else {
                    ESP_LOGW(TAG, "Time sync failed");
                }
            }

            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Cleanup on failure
        vPortFree(fast_telemetry_msg.data.data);
        vPortFree(slow_telemetry_msg.data.data);
        vPortFree(log_msg.data.data);
        
        (void)rcl_publisher_fini(&fast_telemetry_pub, &node);
        (void)rcl_publisher_fini(&slow_telemetry_pub, &node);
        (void)rcl_publisher_fini(&log_pub, &node);
        (void)rcl_subscription_fini(&mode_sub, &node);
        (void)rcl_subscription_fini(&twist_sub, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        (void)rcl_init_options_fini(&init_options);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t uros_manager_start(void) {
    xTaskCreatePinnedToCore(task_uros_core0, "uros_task", 16000, NULL, 5, NULL, 0);
    return ESP_OK;
}
