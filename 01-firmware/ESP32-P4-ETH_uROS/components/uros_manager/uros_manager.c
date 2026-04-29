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
#define EXECUTOR_HANDLES 6

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing...",__LINE__,(int)temp_rc);}}

// =============================================================================
// Static Variables (Private)
// =============================================================================

static rcl_publisher_t line_sensors_pub;
static rcl_publisher_t calibration_pub;
static rcl_publisher_t pid_pub;
static rcl_publisher_t odometry_pub;
static rcl_publisher_t state_pub;
static rcl_publisher_t log_pub;

static rcl_subscription_t mode_sub;
static rcl_subscription_t twist_sub;

static std_msgs__msg__Float32MultiArray line_sensors_msg;
static std_msgs__msg__Float32MultiArray calibration_msg;
static std_msgs__msg__Float32MultiArray pid_msg;
static std_msgs__msg__Float32MultiArray odometry_msg;
static std_msgs__msg__Float32MultiArray state_msg;
static std_msgs__msg__String log_msg;

static std_msgs__msg__Int8 recv_mode_msg;
static geometry_msgs__msg__Twist recv_twist_msg;

// Networking stats
static float g_latency_ms = 0.0f;
static float g_jitter_ms = 0.0f;
static float g_last_latency_ms = -1.0f;

// =============================================================================
// Memory Allocator
// =============================================================================

static void * microros_allocate(size_t size, void * state) { (void)state; return pvPortMalloc(size); }
static void microros_deallocate(void * pointer, void * state) { (void)state; vPortFree(pointer); }
static void * microros_reallocate(void * pointer, size_t size, void * state) { (void)state; return realloc(pointer, size); }
static void * microros_zero_allocate(size_t n, size_t size, void * state) {
    (void)state;
    void * ptr = pvPortMalloc(n * size);
    if (ptr) memset(ptr, 0, n * size);
    return ptr;
}

// =============================================================================
// Callbacks
// =============================================================================

static void mode_callback(const void * msvin) {
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msvin;
    robot_mode_cmd_t cmd = { .new_mode = msg->data };
    shared_memory_push_mode_cmd(&cmd);
    
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    
    snprintf(log_msg.data.data, log_msg.data.capacity, 
             "events,type=MODE_CHANGE,robot=robot_node msg=\"Order received mode change to %d\" %llu", 
             msg->data, ts_ns);
    log_msg.data.size = strlen(log_msg.data.data);
    RCSOFTCHECK(rcl_publish(&log_pub, &log_msg, NULL));
}

static void twist_callback(const void * msvin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
    robot_twist_cmd_t cmd = { .linear_x = msg->linear.x, .linear_y = msg->linear.y, .angular_z = msg->angular.z };
    shared_memory_push_twist_cmd(&cmd);
}

static void timer_fast_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer == NULL) return;

    // 1. Line Sensors
    robot_line_sensors_t line_data;
    if (shared_memory_get_line_sensors(&line_data) == ESP_OK) {
        memcpy(&line_sensors_msg.data.data[0], line_data.raw, 8 * sizeof(float));
        memcpy(&line_sensors_msg.data.data[8], line_data.normalized, 8 * sizeof(float));
        RCSOFTCHECK(rcl_publish(&line_sensors_pub, &line_sensors_msg, NULL));
    }

    // 2. PID Data
    robot_pid_data_t pid_data;
    if (shared_memory_get_pid_data(&pid_data) == ESP_OK) {
        pid_msg.data.data[0] = pid_data.error;
        pid_msg.data.data[1] = pid_data.setpoint;
        pid_msg.data.data[2] = pid_data.output;
        pid_msg.data.data[3] = pid_data.p_term;
        pid_msg.data.data[4] = pid_data.i_term;
        pid_msg.data.data[5] = pid_data.d_term;
        RCSOFTCHECK(rcl_publish(&pid_pub, &pid_msg, NULL));
    }
}

static void timer_slow_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer == NULL) return;

    // 1. Calibration
    robot_calibration_t cal_data;
    if (shared_memory_get_calibration(&cal_data) == ESP_OK) {
        memcpy(&calibration_msg.data.data[0], cal_data.min, 8 * sizeof(float));
        memcpy(&calibration_msg.data.data[8], cal_data.max, 8 * sizeof(float));
        RCSOFTCHECK(rcl_publish(&calibration_pub, &calibration_msg, NULL));
    }

    // 2. Odometry
    robot_odometry_t odom_data;
    if (shared_memory_get_odometry(&odom_data) == ESP_OK) {
        odometry_msg.data.data[0] = odom_data.pos_x;
        odometry_msg.data.data[1] = odom_data.pos_y;
        odometry_msg.data.data[2] = odom_data.theta;
        odometry_msg.data.data[3] = odom_data.vel_linear;
        odometry_msg.data.data[4] = odom_data.vel_angular;
        RCSOFTCHECK(rcl_publish(&odometry_pub, &odometry_msg, NULL));
    }

    // 3. State
    robot_state_t state;
    if (shared_memory_get_state(&state) == ESP_OK) {
        state_msg.data.data[0] = (float)state.current_mode;
        state_msg.data.data[1] = state.battery_voltage;
        state_msg.data.data[2] = g_latency_ms;
        state_msg.data.data[3] = g_jitter_ms;
        state_msg.data.data[4] = (float)state.uptime_s;
        RCSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));
    }
}

// =============================================================================
// micro-ROS Task
// =============================================================================

static void task_uros_core0(void * arg) {
    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;

    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer_fast, timer_slow;
    rclc_executor_t executor;

    while(1) {
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif

        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        node = rcl_get_zero_initialized_node();
        RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

        // Publishers
        RCCHECK(rclc_publisher_init_default(&line_sensors_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/line_sensors"));
        RCCHECK(rclc_publisher_init_default(&calibration_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/calibration"));
        RCCHECK(rclc_publisher_init_default(&pid_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/pid"));
        RCCHECK(rclc_publisher_init_default(&odometry_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/odometry"));
        RCCHECK(rclc_publisher_init_default(&state_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/robot/state"));
        RCCHECK(rclc_publisher_init_default(&log_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot/logs"));

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

        // Buffers
        line_sensors_msg.data.data = (float*)pvPortMalloc(16 * sizeof(float)); line_sensors_msg.data.capacity = line_sensors_msg.data.size = 16;
        calibration_msg.data.data = (float*)pvPortMalloc(16 * sizeof(float)); calibration_msg.data.capacity = calibration_msg.data.size = 16;
        pid_msg.data.data = (float*)pvPortMalloc(6 * sizeof(float)); pid_msg.data.capacity = pid_msg.data.size = 6;
        odometry_msg.data.data = (float*)pvPortMalloc(5 * sizeof(float)); odometry_msg.data.capacity = odometry_msg.data.size = 5;
        state_msg.data.data = (float*)pvPortMalloc(5 * sizeof(float)); state_msg.data.capacity = state_msg.data.size = 5;
        char log_buffer[128]; log_msg.data.data = log_buffer; log_msg.data.capacity = 128;

        int64_t last_sync_time = 0;

        while(1) {
            int64_t now = esp_timer_get_time();
            if ((now - last_sync_time) > (SYNC_INTERVAL_MS * 1000)) {
                if (rmw_uros_sync_session(1000) == RCL_RET_OK) {
                    int64_t t_before = esp_timer_get_time();
                    int64_t agent_ms = rmw_uros_epoch_millis();
                    g_latency_ms = (float)(esp_timer_get_time() - t_before) / 2000.0f;
                    
                    if (g_last_latency_ms >= 0) {
                        g_jitter_ms = fabsf(g_latency_ms - g_last_latency_ms);
                    }
                    g_last_latency_ms = g_latency_ms;

                    struct timeval tv_sync = { .tv_sec = agent_ms / 1000, .tv_usec = (agent_ms % 1000) * 1000 };
                    settimeofday(&tv_sync, NULL);
                    last_sync_time = esp_timer_get_time();
                }
            }
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vPortFree(line_sensors_msg.data.data); vPortFree(calibration_msg.data.data);
        vPortFree(pid_msg.data.data); vPortFree(odometry_msg.data.data); vPortFree(state_msg.data.data);
        rcl_publisher_fini(&line_sensors_pub, &node); rcl_publisher_fini(&calibration_pub, &node);
        rcl_publisher_fini(&pid_pub, &node); rcl_publisher_fini(&odometry_pub, &node);
        rcl_publisher_fini(&state_pub, &node); rcl_publisher_fini(&log_pub, &node);
        rcl_subscription_fini(&mode_sub, &node); rcl_subscription_fini(&twist_sub, &node);
        rcl_node_fini(&node); rclc_support_fini(&support); rcl_init_options_fini(&init_options);
    }
}

esp_err_t uros_manager_start(void) {
    xTaskCreatePinnedToCore(task_uros_core0, "uros_task", 16384, NULL, 5, NULL, 0);
    return ESP_OK;
}
