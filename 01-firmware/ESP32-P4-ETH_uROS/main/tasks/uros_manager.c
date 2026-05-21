#include "uros_manager.h"
#include "shared_memory.h"
#include "state_machine.h"
#include "audio_player.h"

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

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "UROS_MGR";

#define STRING_BUFFER_LEN 256
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

// --- Publishers ---
static rcl_publisher_t diag_publisher;
static rcl_publisher_t voltage_publisher;
static rcl_publisher_t sensors_publisher;
static rcl_publisher_t motors_publisher;
static rcl_publisher_t status_publisher;

// --- Subscriptions ---
static rcl_subscription_t velocity_subscriber;
static rcl_subscription_t mode_subscriber;
static rcl_subscription_t config_subscriber;
static rcl_subscription_t curvature_subscriber;
static rcl_subscription_t pid_subscriber;

// --- Messages ---
static std_msgs__msg__String diag_msg;
static std_msgs__msg__Float32 voltage_msg;
static std_msgs__msg__String sensors_msg;
static std_msgs__msg__String motors_msg;
static std_msgs__msg__String status_msg;
static geometry_msgs__msg__Twist cmd_vel;
static std_msgs__msg__Int8 mode_msg;
static std_msgs__msg__String config_msg;
static std_msgs__msg__Float32 curvature_msg;
static std_msgs__msg__String pid_msg;

// --- Shared state ---
static float g_latency_ms = 0.0f;
static float g_offset_ms = 0.0f;
static float g_jitter_ms = 0.0f;
static float g_last_latency_ms = -1.0f;

// ============================================================
// CALLBACKS
// ============================================================

static void mode_callback(const void *msvin)
{
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msvin;
    int8_t mode_id = msg->data;
    if (mode_id == 10) {
        audio_player_play(INTHEEND);
        ESP_LOGI(TAG, "Playing fight sound");
        return;
    } else if (mode_id == 11) {
        audio_player_play(TOKYO);
        ESP_LOGI(TAG, "Playing drift sound");
        return;
    } else if (mode_id == 12) {
        audio_player_play(HOLA);
        ESP_LOGI(TAG, "Playing hola sound");
        return;
    } else if (mode_id == 13) {
        audio_player_play(DEMACIA);
        ESP_LOGI(TAG, "Playing demacia sound");
        return;
    } else if (mode_id < 0 || mode_id >= MODE_COUNT) {
        ESP_LOGW(TAG, "Ignoring invalid mode id: %d", mode_id);
        return;
    }
    bool accepted = state_machine_request_mode((robot_mode_t)mode_id, true);
    ESP_LOGI(TAG, "Mode change request -> %d (%s)", mode_id, accepted ? "accepted" : "rejected");
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
    float target_l = v - ((w * WHEEL_BASE_M) / 2.0f);
    float target_r = v + ((w * WHEEL_BASE_M) / 2.0f);
    uint32_t now = millis_now();

    shared_memory_t *shm = shared_memory_get();
    if (shm == NULL) {
        ESP_LOGW(TAG, "Shared memory unavailable, dropping cmd_vel");
        return;
    }
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        shm->teleop.target_speed_left = target_l;
        shm->teleop.target_speed_right = target_r;
        shm->teleop.last_update_ms = now;
        xSemaphoreGive(shm->mutex);
    } else {
        ESP_LOGW(TAG, "Shared memory busy, dropping cmd_vel");
    }
    ESP_LOGI(TAG, "cmd_vel -> lin.x: %.2f, ang.z: %.2f", (float)msg->linear.x, (float)msg->angular.z);
}

static void config_callback(const void *msvin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msvin;
    ESP_LOGI(TAG, "Config received: %s", msg->data.data);
}

static void curvature_callback(const void *msvin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msvin;
    ESP_LOGI(TAG, "Curvature received: %.3f", (float)msg->data);
}

static void pid_callback(const void *msvin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msvin;
    ESP_LOGI(TAG, "PID received: %s", msg->data.data);
}

// ============================================================
// TIMER CALLBACKS
// ============================================================

static void diag_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);
    char strftime_buf[32];
    strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);

    snprintf(diag_msg.data.data, diag_msg.data.capacity,
             "{\"ts\":\"%s.%06ld\",\"lat\":%.3f,\"off\":%.3f,\"jit\":%.3f}",
             strftime_buf, (long)tv.tv_usec, g_latency_ms, g_offset_ms, g_jitter_ms);
    diag_msg.data.size = strlen(diag_msg.data.data);
    RCSOFTCHECK(rcl_publish(&diag_publisher, &diag_msg, NULL));

    // Also publish voltage
    shared_memory_t *shm = shared_memory_get();
    if (shm && xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        voltage_msg.data = shm->sensors.battery_voltage;
        xSemaphoreGive(shm->mutex);
    }
    RCSOFTCHECK(rcl_publish(&voltage_publisher, &voltage_msg, NULL));
}

static void telemetry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    shared_memory_t *shm = shared_memory_get();
    if (shm == NULL) return;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;

    // Pack sensors as JSON
    snprintf(sensors_msg.data.data, sensors_msg.data.capacity,
        "{\"sl\":%.4f,\"sr\":%.4f,\"dl\":%.4f,\"dr\":%.4f,\"bat\":%.2f,\"cur\":%.2f,"
        "\"lp\":%.4f,\"ld\":%d,\"lc\":%d,"
        "\"n0\":%.3f,\"n1\":%.3f,\"n2\":%.3f,\"n3\":%.3f,\"n4\":%.3f,\"n5\":%.3f,\"n6\":%.3f,\"n7\":%.3f}",
        shm->sensors.motor_speed_left, shm->sensors.motor_speed_right,
        shm->sensors.motor_distance_left, shm->sensors.motor_distance_right,
        shm->sensors.battery_voltage, shm->sensors.robot_current,
        shm->sensors.line_position_m, shm->sensors.line_detected ? 1 : 0,
        shm->sensors.line_is_calibrated ? 1 : 0,
        shm->sensors.line_norm[0], shm->sensors.line_norm[1],
        shm->sensors.line_norm[2], shm->sensors.line_norm[3],
        shm->sensors.line_norm[4], shm->sensors.line_norm[5],
        shm->sensors.line_norm[6], shm->sensors.line_norm[7]);
    sensors_msg.data.size = strlen(sensors_msg.data.data);

    // Pack motors as JSON
    snprintf(motors_msg.data.data, motors_msg.data.capacity,
        "{\"tl\":%.4f,\"tr\":%.4f,\"al\":%.4f,\"ar\":%.4f}",
        shm->sensors.target_speed_left, shm->sensors.target_speed_right,
        shm->sensors.motor_speed_left, shm->sensors.motor_speed_right);
    motors_msg.data.size = strlen(motors_msg.data.data);

    xSemaphoreGive(shm->mutex);

    // Pack status as JSON
    snprintf(status_msg.data.data, status_msg.data.capacity,
        "{\"up\":%lu,\"mode\":%d}",
        (unsigned long)(esp_timer_get_time() / 1000000),
        (int)state_machine_get_context()->current_mode);
    status_msg.data.size = strlen(status_msg.data.data);

    // Publish all three
    RCSOFTCHECK(rcl_publish(&sensors_publisher, &sensors_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motors_publisher, &motors_msg, NULL));
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
}

// ============================================================
// MAIN MICRO-ROS TASK
// ============================================================

static void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro_ros_task started (sumo_5 build)");
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();

    while (1) {
        rclc_support_t support;
        rcl_node_t node;
        rcl_timer_t diag_timer;
        rcl_timer_t telemetry_timer;
        rclc_executor_t executor;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        ESP_LOGI(TAG, "calling rcl_init_options_init");
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED init_options_init");
            return;
        }
        ESP_LOGI(TAG, "OK init_options_init");

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        if (rmw_options == NULL) {
            ESP_LOGE(TAG, "FAILED get_rmw_init_options (NULL)");
            return;
        }
        ESP_LOGI(TAG, "set udp addr %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rmw_options);
#endif

        ESP_LOGI(TAG, "calling support_init (connect %s:%s)...",
            CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            { rcl_ret_t _r = rcl_init_options_fini(&init_options); (void)_r; }
            ESP_LOGE(TAG, "FAILED support init");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "OK support init");

        node = rcl_get_zero_initialized_node();
        if (rclc_node_init_default(&node, "esp32_p4_robot", "", &support) != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED node init");
            return;
        }
        ESP_LOGI(TAG, "OK node init");

        // === 5 PUBLISHERS ===

        // 1. Diagnostics (String)
        // Use unique topic name to avoid DDS conflicts with other robots
        rcl_ret_t rc_pub = rclc_publisher_init_default(&diag_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/microROS/esp_diag_time");
        if (rc_pub != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED diag pub: ret=%d, node_ok=%d, ts_ok=%d",
                (int)rc_pub,
                (int)rcl_node_is_valid(&node),
                (int)(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String) != NULL));
            return;
        }
        ESP_LOGI(TAG, "OK diag pub");

        if (rclc_publisher_init_default(&voltage_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/robot/voltage") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED voltage pub");
            return;
        }
        ESP_LOGI(TAG, "OK voltage pub");

        // 3. Sensors (String - JSON)
        if (rclc_publisher_init_default(&sensors_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/robot/sensors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED sensors pub");
            return;
        }
        ESP_LOGI(TAG, "OK sensors pub");

        // 4. Motors (String - JSON)
        if (rclc_publisher_init_default(&motors_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/robot/motors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED motors pub");
            return;
        }
        ESP_LOGI(TAG, "OK motors pub");

        // 5. Status (String - JSON)
        if (rclc_publisher_init_default(&status_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/robot/status") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED status pub");
            return;
        }
        ESP_LOGI(TAG, "OK status pub");

        // === 5 SUBSCRIPTIONS ===

        // 1. cmd_vel (Twist)
        RCCHECK(rclc_subscription_init_default(
            &velocity_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"));

        // 2. mode_cmd (Int8)
        RCCHECK(rclc_subscription_init_default(
            &mode_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "/robot/mode_cmd"));

        // 3. config (String - JSON)
        if (rclc_subscription_init_default(&config_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/robot/config") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED config sub");
            return;
        }
        ESP_LOGI(TAG, "OK config sub");

        // 4. curvature (Float32)
        if (rclc_subscription_init_default(&curvature_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/robot/curvature") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED curvature sub");
            return;
        }
        ESP_LOGI(TAG, "OK curvature sub");

        // 5. pid_motors (String - JSON)
        if (rclc_subscription_init_default(&pid_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/robot/pid_motors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED pid sub");
            return;
        }
        ESP_LOGI(TAG, "OK pid sub");

        // === 2 TIMERS ===

        RCCHECK(rclc_timer_init_default2(
            &diag_timer, &support,
            RCL_MS_TO_NS(5000),
            diag_timer_callback, true));

        RCCHECK(rclc_timer_init_default2(
            &telemetry_timer, &support,
            RCL_MS_TO_NS(50),
            telemetry_timer_callback, true));

        // === EXECUTOR (7 handles: 2 timers + 5 subscriptions) ===

        if (rclc_executor_init(&executor, &support.context, 7, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED executor init");
            return;
        }
        ESP_LOGI(TAG, "OK executor init (7 handles)");

        RCCHECK(rclc_executor_add_timer(&executor, &diag_timer));
        RCCHECK(rclc_executor_add_timer(&executor, &telemetry_timer));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &velocity_subscriber, &cmd_vel,
            &subscription_vel_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &mode_subscriber, &mode_msg,
            &mode_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &config_subscriber, &config_msg,
            &config_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &curvature_subscriber, &curvature_msg,
            &curvature_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &pid_subscriber, &pid_msg,
            &pid_callback, ON_NEW_DATA));

        // Init message buffers
        static char diag_buffer[STRING_BUFFER_LEN];
        diag_msg.data.data = diag_buffer;
        diag_msg.data.capacity = STRING_BUFFER_LEN;
        diag_msg.data.size = 0;

        static char sensors_buffer[STRING_BUFFER_LEN];
        sensors_msg.data.data = sensors_buffer;
        sensors_msg.data.capacity = STRING_BUFFER_LEN;
        sensors_msg.data.size = 0;

        static char motors_buffer[STRING_BUFFER_LEN];
        motors_msg.data.data = motors_buffer;
        motors_msg.data.capacity = STRING_BUFFER_LEN;
        motors_msg.data.size = 0;

        static char status_buffer[STRING_BUFFER_LEN];
        status_msg.data.data = status_buffer;
        status_msg.data.capacity = STRING_BUFFER_LEN;
        status_msg.data.size = 0;

        static char config_buffer[STRING_BUFFER_LEN];
        config_msg.data.data = config_buffer;
        config_msg.data.capacity = STRING_BUFFER_LEN;
        config_msg.data.size = 0;

        static char pid_buffer[STRING_BUFFER_LEN];
        pid_msg.data.data = pid_buffer;
        pid_msg.data.capacity = STRING_BUFFER_LEN;
        pid_msg.data.size = 0;

        voltage_msg.data = 0.0f;
        curvature_msg.data = 0.0f;

        int64_t last_sync_time = 0;

        while (1) {
            int64_t now = esp_timer_get_time();

            if ((now - last_sync_time) > (SYNC_INTERVAL_MS * 1000)) {
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
                int64_t t_before = esp_timer_get_time();
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
                    int64_t local_us = (int64_t)tv_now.tv_sec * 1000000 + tv_now.tv_usec;
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

            rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
                ESP_LOGW(TAG, "spin_some error %d, continuing", (int)spin_ret);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Cleanup
        { rcl_ret_t _r = rcl_publisher_fini(&diag_publisher, &node); (void)_r; }
        { rcl_ret_t _r = rcl_publisher_fini(&voltage_publisher, &node); (void)_r; }
        { rcl_ret_t _r = rcl_publisher_fini(&sensors_publisher, &node); (void)_r; }
        { rcl_ret_t _r = rcl_publisher_fini(&motors_publisher, &node); (void)_r; }
        { rcl_ret_t _r = rcl_publisher_fini(&status_publisher, &node); (void)_r; }

        { rcl_ret_t _r = rcl_subscription_fini(&velocity_subscriber, &node); (void)_r; }
        { rcl_ret_t _r = rcl_subscription_fini(&mode_subscriber, &node); (void)_r; }
        { rcl_ret_t _r = rcl_subscription_fini(&config_subscriber, &node); (void)_r; }
        { rcl_ret_t _r = rcl_subscription_fini(&curvature_subscriber, &node); (void)_r; }
        { rcl_ret_t _r = rcl_subscription_fini(&pid_subscriber, &node); (void)_r; }

        { rcl_ret_t _r = rcl_timer_fini(&diag_timer); (void)_r; }
        { rcl_ret_t _r = rcl_timer_fini(&telemetry_timer); (void)_r; }
        { rcl_ret_t _r = rclc_executor_fini(&executor); (void)_r; }
        { rcl_ret_t _r = rcl_node_fini(&node); (void)_r; }
        { rcl_ret_t _r = rclc_support_fini(&support); (void)_r; }
        { rcl_ret_t _r = rcl_init_options_fini(&init_options); (void)_r; }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t uros_manager_start(void)
{
    static bool started = false;
    ESP_LOGI(TAG, "Inicio micro-ROS (sumo_5)");
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
        NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS task");
        return ESP_FAIL;
    }
    started = true;
    return ESP_OK;
}
