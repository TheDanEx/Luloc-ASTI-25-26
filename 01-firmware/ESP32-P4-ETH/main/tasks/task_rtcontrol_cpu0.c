#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"
#include "motor.h"


static const char *TAG = "rt_cntrl";

static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_25, .in2 = GPIO_NUM_26},
    .right = { .in1 = GPIO_NUM_23, .in2 = GPIO_NUM_5},

    .nsleep = GPIO_NUM_NC,      // o el GPIO real
    .pwm_hz = 20000,
    .resolution_hz = 10000000,  // 10 MHz
    .deadband = 30,
    .brake_on_stop = true,
};

static void task_rtcontrol_cpu0(void *arg)
{
    ESP_LOGD(TAG, "Control loop running");
    motor_mcpwm_init(&motors);

    while(1) {
        // Motor control, sensor reading, real-time algorithms
        //ESP_LOGI(TAG, "Turning th motor %s!", "FORWARD");
        motor_mcpwm_set(&motors, 700, 700);   // adelante
        vTaskDelay(pdMS_TO_TICKS(1500));
        // ESP_LOGI(TAG, "Turning the motor %s!", "STOP");
        // motor_mcpwm_stop(&motors);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // ESP_LOGI(TAG, "Turning the motor %s!", "REVERSE");
        // motor_mcpwm_set(&motors, -700, -700);   // atras
        // vTaskDelay(pdMS_TO_TICKS(1500));
        // ESP_LOGI(TAG, "Turning the motor %s!", "STOP");
        // motor_mcpwm_stop(&motors);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // vTaskDelay(pdMS_TO_TICKS(1));  // 100 Hz
    }
}

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
