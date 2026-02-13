#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"

static const char *TAG = "rt_cntrl";

static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_18, .in2 = GPIO_NUM_19, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_0 },
    .right = { .in1 = GPIO_NUM_23, .in2 = GPIO_NUM_5,  .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_1 },

    .nsleep = GPIO_NUM_NC,   // o un GPIO si lo controlas
    .pwm_hz = 20000,
    .deadband = 30,
    .brake_on_stop = false,
};

static void task_rtcontrol_cpu0(void *arg)
{
    ESP_LOGD(TAG, "Control loop running");
    motor_mcpwm_init(&motors);

    while(1) {
        // Motor control, sensor reading, real-time algorithms
        ESP_LOGI(TAG, "Turning the motor %s!", "FORWARD");
        motor_mcpwm_set(&motors, 1000, 1000);   // adelante
        vTaskDelay(pdMS_TO_TICKS(1500));
        ESP_LOGI(TAG, "Turning the motor %s!", "STOP");
        motor_mcpwm_stop(&motors);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Turning the motor %s!", "REVERSE");
        motor_mcpwm_set(&motors, -1000, -1000); // atrás
        vTaskDelay(pdMS_TO_TICKS(1500));
        ESP_LOGI(TAG, "Turning the motor %s!", "STOP");
        motor_mcpwm_stop(&motors);
        vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(1));  // 100 Hz
    }
}

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
