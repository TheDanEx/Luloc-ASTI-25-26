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

#define STRING_BUFFER_LEN 1024
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
    if ( mode_id == 10){
        audio_player_play(INTHEEND);
        ESP_LOGI(TAG, "Playing fight sound");
        return;
    }else if ( mode_id == 11){
        audio_player_play(TOKYO);
        ESP_LOGI(TAG, "Playing drift sound");
        return;
    }else if ( mode_id == 12){
        audio_player_play(HOLA);
        ESP_LOGI(TAG, "Playing hola sound");
        return;
    }else if ( mode_id == 13){
        audio_player_play(DEMACIA);
        ESP_LOGI(TAG, "Playing demacia sound");
        return;
    }else if ( mode_id == 14){
        audio_player_play(ROUND_1);
        ESP_LOGI(TAG, "Playing round 1 sound");
        mode_id = 7;
    }else if ( mode_id == 15){
        audio_player_play(ROUND_2);
        ESP_LOGI(TAG, "Playing round 2 sound");
        mode_id = 7;
    }else if ( mode_id == 16){
        audio_player_play(ROUND_3);
        ESP_LOGI(TAG, "Playing round 3 sound");
        mode_id = 7;
    }else if (mode_id < 0 || mode_id >= MODE_COUNT) {
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

    cmd_vel_item_t item = {
        .speed_left   = v - ((w * WHEEL_BASE_M) / 2.0f),
        .speed_right  = v + ((w * WHEEL_BASE_M) / 2.0f),
        .timestamp_ms = millis_now()
    };

    xQueueOverwrite(g_cmd_vel_queue, &item);
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

    robot_sensor_data_t snap;
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
    memcpy(&snap, &shm->sensors, sizeof(snap));
    xSemaphoreGive(shm->mutex);

    // Pack sensors as JSON
    snprintf(sensors_msg.data.data, sensors_msg.data.capacity,
        "{\"sl\":%.4f,\"sr\":%.4f,\"dl\":%.4f,\"dr\":%.4f,\"bat\":%.2f,\"cur\":%.2f,"
        "\"lp\":%.4f,\"ld\":%d,\"lc\":%d,"
        "\"n0\":%.3f,\"n1\":%.3f,\"n2\":%.3f,\"n3\":%.3f,\"n4\":%.3f,\"n5\":%.3f,\"n6\":%.3f,\"n7\":%.3f,"
        "\"r0\":%u,\"r1\":%u,\"r2\":%u,\"r3\":%u,\"r4\":%u,\"r5\":%u,\"r6\":%u,\"r7\":%u,"
        "\"min0\":%u,\"min1\":%u,\"min2\":%u,\"min3\":%u,\"min4\":%u,\"min5\":%u,\"min6\":%u,\"min7\":%u,"
        "\"max0\":%u,\"max1\":%u,\"max2\":%u,\"max3\":%u,\"max4\":%u,\"max5\":%u,\"max6\":%u,\"max7\":%u,"
        "\"tmp\":%.1f}",
        snap.motor_speed_left, snap.motor_speed_right,
        snap.motor_distance_left, snap.motor_distance_right,
        snap.battery_voltage, snap.robot_current,
        snap.line_position_m, snap.line_detected ? 1 : 0,
        snap.line_is_calibrated ? 1 : 0,
        snap.line_norm[0], snap.line_norm[1],
        snap.line_norm[2], snap.line_norm[3],
        snap.line_norm[4], snap.line_norm[5],
        snap.line_norm[6], snap.line_norm[7],
        (unsigned)snap.line_raw[0], (unsigned)snap.line_raw[1],
        (unsigned)snap.line_raw[2], (unsigned)snap.line_raw[3],
        (unsigned)snap.line_raw[4], (unsigned)snap.line_raw[5],
        (unsigned)snap.line_raw[6], (unsigned)snap.line_raw[7],
        (unsigned)snap.line_min[0], (unsigned)snap.line_min[1],
        (unsigned)snap.line_min[2], (unsigned)snap.line_min[3],
        (unsigned)snap.line_min[4], (unsigned)snap.line_min[5],
        (unsigned)snap.line_min[6], (unsigned)snap.line_min[7],
        (unsigned)snap.line_max[0], (unsigned)snap.line_max[1],
        (unsigned)snap.line_max[2], (unsigned)snap.line_max[3],
        (unsigned)snap.line_max[4], (unsigned)snap.line_max[5],
        (unsigned)snap.line_max[6], (unsigned)snap.line_max[7],
        snap.temperature);
    sensors_msg.data.size = strlen(sensors_msg.data.data);

    // Pack motors as JSON (target/actual speeds + per-motor PID voltage breakdown)
    snprintf(motors_msg.data.data, motors_msg.data.capacity,
        "{\"tl\":%.4f,\"tr\":%.4f,\"al\":%.4f,\"ar\":%.4f,"
        "\"ffl\":%.3f,\"pl\":%.3f,\"il\":%.3f,\"dl\":%.3f,"
        "\"ffr\":%.3f,\"pr\":%.3f,\"ir\":%.3f,\"dr\":%.3f}",
        snap.target_speed_left, snap.target_speed_right,
        snap.motor_speed_left, snap.motor_speed_right,
        snap.motor_pid_ff_l, snap.motor_pid_p_l,
        snap.motor_pid_i_l, snap.motor_pid_d_l,
        snap.motor_pid_ff_r, snap.motor_pid_p_r,
        snap.motor_pid_i_r, snap.motor_pid_d_r);
    motors_msg.data.size = strlen(motors_msg.data.data);

    // Snapshot cycle timing while holding mutex
    float cyc_mean = snap.cycle_mean_us;
    float cyc_min  = snap.cycle_min_us;
    float cyc_max  = snap.cycle_max_us;
    float cyc_p95  = snap.cycle_p95_us;
    uint32_t cyc_over = snap.cycle_overruns;
    float busy_mean = snap.busy_mean_us;
    float busy_min  = snap.busy_min_us;
    float busy_max  = snap.busy_max_us;
    float busy_p95  = snap.busy_p95_us;

    float cyc_hz = (cyc_mean > 0.0f) ? 1000000.0f / cyc_mean : 0.0f;
    float busy_hz = (busy_mean > 0.0f) ? 1000000.0f / busy_mean : 0.0f;

    // Pack status as JSON
    snprintf(status_msg.data.data, status_msg.data.capacity,
        "{\"up\":%lu,\"mode\":%d,\"cyc_m\":%.0f,\"cyc_x\":%.0f,\"cyc_n\":%.0f,\"cyc_p\":%.0f,\"cyc_o\":%lu,"
        "\"busy_m\":%.0f,\"busy_x\":%.0f,\"busy_n\":%.0f,\"busy_p\":%.0f,"
        "\"cyc_h\":%.0f,\"busy_h\":%.0f}",
        (unsigned long)(esp_timer_get_time() / 1000000),
        (int)state_machine_get_context()->current_mode,
        cyc_mean, cyc_max, cyc_min, cyc_p95,
        (unsigned long)cyc_over,
        busy_mean, busy_max, busy_min, busy_p95,
        cyc_hz, busy_hz);
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
    uint32_t reconnect_delay_ms = 2000;

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
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK init_options_init");

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        if (rmw_options == NULL) {
            ESP_LOGE(TAG, "FAILED get_rmw_init_options (NULL)");
            goto cleanup_reconnect;
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
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK node init");

        // === 5 PUBLISHERS ===

        // 1. Diagnostics (String)
        // Use unique topic name to avoid DDS conflicts with other robots
        rcl_ret_t rc_pub = rclc_publisher_init_default(&diag_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/microROS/esp_diag_time");
        if (rc_pub != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED diag pub: ret=%d", (int)rc_pub);
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK diag pub");

        if (rclc_publisher_init_default(&voltage_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/robot/voltage") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED voltage pub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK voltage pub");

        // 3. Sensors (String - JSON)
        if (rclc_publisher_init_default(&sensors_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/sensors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED sensors pub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK sensors pub");

        // 4. Motors (String - JSON)
        if (rclc_publisher_init_default(&motors_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/motors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED motors pub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK motors pub");

        // 5. Status (String - JSON)
        if (rclc_publisher_init_default(&status_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/status") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED status pub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK status pub");

        // === 5 SUBSCRIPTIONS ===

        // 1. cmd_vel (Twist)
        rcl_ret_t rc_sub;
        rc_sub = rclc_subscription_init_default(
            &velocity_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel");
        if (rc_sub != RCL_RET_OK) { ESP_LOGE(TAG, "FAILED cmd_vel sub"); goto cleanup_reconnect; }

        // 2. mode_cmd (Int8)
        rc_sub = rclc_subscription_init_default(
            &mode_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "/robot/mode_cmd");
        if (rc_sub != RCL_RET_OK) { ESP_LOGE(TAG, "FAILED mode sub"); goto cleanup_reconnect; }

        // 3. config (String - JSON)
        if (rclc_subscription_init_default(&config_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/config") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED config sub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK config sub");

        // 4. curvature (Float32)
        if (rclc_subscription_init_default(&curvature_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/curvature") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED curvature sub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK curvature sub");

        // 5. pid_motors (String - JSON)
        if (rclc_subscription_init_default(&pid_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/pid_motors") != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED pid sub");
            goto cleanup_reconnect;
        }
        ESP_LOGI(TAG, "OK pid sub");

        // === 2 TIMERS ===

        rcl_ret_t rc_timer;
        rc_timer = rclc_timer_init_default2(
            &diag_timer, &support,
            RCL_MS_TO_NS(5000),
            diag_timer_callback, true);
        if (rc_timer != RCL_RET_OK) { ESP_LOGE(TAG, "FAILED diag timer"); goto cleanup_reconnect; }

        rc_timer = rclc_timer_init_default2(
            &telemetry_timer, &support,
            RCL_MS_TO_NS(50),
            telemetry_timer_callback, true);
        if (rc_timer != RCL_RET_OK) { ESP_LOGE(TAG, "FAILED telemetry timer"); goto cleanup_reconnect; }

        // === EXECUTOR (7 handles: 2 timers + 5 subscriptions) ===

        if (rclc_executor_init(&executor, &support.context, 7, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "FAILED executor init");
            goto cleanup_reconnect;
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
        int spin_errors = 0;

        while (spin_errors < 5) {
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
            if (spin_ret == RCL_RET_OK || spin_ret == RCL_RET_TIMEOUT) {
                spin_errors = 0;
            } else {
                spin_errors++;
                if (spin_errors == 1) {
                    ESP_LOGW(TAG, "uROS spin error %d, monitoring...", (int)spin_ret);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGE(TAG, "uROS agent lost, reconnecting in %lums...", (unsigned long)reconnect_delay_ms);

cleanup_reconnect:
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

        vTaskDelay(pdMS_TO_TICKS(reconnect_delay_ms));
        if (reconnect_delay_ms < 30000) reconnect_delay_ms *= 2;
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
