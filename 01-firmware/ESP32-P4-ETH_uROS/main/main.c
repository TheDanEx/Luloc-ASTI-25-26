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

// Micro-ROS headers
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define STRING_BUFFER_LEN 160
#define SYNC_INTERVAL_MS 5000 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing...\n",__LINE__,(int)temp_rc);}}

// Global ROS 2 entities
rcl_publisher_t diag_publisher;
rcl_publisher_t my_publisher;
rcl_subscription_t my_subscriber;
std_msgs__msg__String diag_msg;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

// Diagnostics variables
float g_latency_ms = 0.0f;
float g_offset_ms = 0.0f;
float g_jitter_ms = 0.0f;
float g_last_latency_ms = -1.0f;

// Callback for subscriber
void subscription_callback(const void * msvin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
	printf("Received from topic: %ld\n", (long)msg->data);
}

// Callback for timer: Periodic execution
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		// 1. Publish Diagnostics String
		struct timeval tv;
		gettimeofday(&tv, NULL); 
		struct tm timeinfo;
		localtime_r(&tv.tv_sec, &timeinfo);
		char strftime_buf[32];
		strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
		
		sprintf(diag_msg.data.data, 
                "[SYNC] %s.%06ld | Lat: %.3fms | Off: %.3fms | Jit: %.3fms", 
                strftime_buf, (long)tv.tv_usec,
                g_latency_ms, g_offset_ms, g_jitter_ms);
		diag_msg.data.size = strlen(diag_msg.data.data);
		
		RCSOFTCHECK(rcl_publish(&diag_publisher, &diag_msg, NULL));
		printf("DIAG: %s\n", diag_msg.data.data);

		// 2. Publish Example Data
		send_msg.data++;
		RCSOFTCHECK(rcl_publish(&my_publisher, &send_msg, NULL));
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_node_t node;
	rcl_timer_t timer;
	rclc_executor_t executor;

	while(1) {
		// 1. Initialize Options
		rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		RCCHECK(rcl_init_options_init(&init_options, allocator));
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
		rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
		rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif

		// 2. Support Init
		if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
			rcl_ret_t f_ret = rcl_init_options_fini(&init_options); (void)f_ret;
			vTaskDelay(pdMS_TO_TICKS(2000));
			continue;
		}

		// 3. Node Init
		node = rcl_get_zero_initialized_node();
		RCCHECK(rclc_node_init_default(&node, "esp32_p4_robot", "", &support));

		// 4. Publishers Init
		RCCHECK(rclc_publisher_init_default(&diag_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/esp_diag_time"));
		RCCHECK(rclc_publisher_init_default(&my_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/publisher"));

		// 5. Subscriber Init
		RCCHECK(rclc_subscription_init_default(&my_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/subscriber"));

		// 6. Timer Init (2 seconds)
		RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(2000), timer_callback, true));

		// 7. Executor Init (3 handles: timer + subscriber + empty)
		RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
		RCCHECK(rclc_executor_add_timer(&executor, &timer));
		RCCHECK(rclc_executor_add_subscription(&executor, &my_subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

		// Setup buffers for diagnostic string
		char buffer[STRING_BUFFER_LEN];
		diag_msg.data.data = buffer;
		diag_msg.data.capacity = STRING_BUFFER_LEN;
		send_msg.data = 0;

        int64_t last_sync_time = 0;

		// 8. Main Loop
		while(1){
            // --- SYNC SESSION LOGIC ---
            int64_t now = esp_timer_get_time();
            if ((now - last_sync_time) > (SYNC_INTERVAL_MS * 1000)) {
                int64_t t_before = esp_timer_get_time();
                if (rmw_uros_sync_session(1000) == RCL_RET_OK) {
                    int64_t t_after = esp_timer_get_time();
                    int64_t rtt_us = t_after - t_before;
                    g_latency_ms = (float)rtt_us / 2000.0f;
                    if (g_last_latency_ms >= 0) g_jitter_ms = fabsf(g_latency_ms - g_last_latency_ms);
                    g_last_latency_ms = g_latency_ms;

                    int64_t agent_ms = rmw_uros_epoch_millis();
                    int64_t agent_us = (agent_ms * 1000) + (rtt_us / 2);
                    struct timeval tv_now;
                    gettimeofday(&tv_now, NULL);
                    int64_t local_us = (int64_t)tv_now.tv_sec * 1000000 + tv_now.tv_usec;
                    g_offset_ms = (float)(agent_us - local_us) / 1000.0f;
                    
                    struct timeval tv_sync = { .tv_sec = agent_us / 1000000, .tv_usec = agent_us % 1000000 };
                    settimeofday(&tv_sync, NULL);
                    last_sync_time = esp_timer_get_time();
                }
            }

			if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) break;
			vTaskDelay(pdMS_TO_TICKS(10));
		}

		// Cleanup
		rcl_publisher_fini(&diag_publisher, &node);
		rcl_publisher_fini(&my_publisher, &node);
		rcl_subscription_fini(&my_subscriber, &node);
		rcl_node_fini(&node);
		rclc_support_fini(&support);
		rcl_init_options_fini(&init_options);
	}
}

void app_main(void)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    xTaskCreate(micro_ros_task, "uros_task", 16384, NULL, 5, NULL);
}
