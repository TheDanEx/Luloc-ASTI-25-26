#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"
#include "motor.h"
#include "shared_memory.h"
#include "pid_siguelineas.h"


static const char *TAG = "rt_cntrl";

#define CONTROL_HZ                    100
#define CONTROL_PERIOD_MS             (1000 / CONTROL_HZ)
#define BASE_SPEED_CMD                (-650)
#define MAX_MOTOR_CMD                 1000

// TODO: Ajustar estos pines según tu cableado real de sensor de línea.
#define LINE_SENSOR_L2_PIN            GPIO_NUM_32
#define LINE_SENSOR_L1_PIN            GPIO_NUM_33
#define LINE_SENSOR_C_PIN             GPIO_NUM_34
#define LINE_SENSOR_R1_PIN            GPIO_NUM_35
#define LINE_SENSOR_R2_PIN            GPIO_NUM_36

static pid_siguelineas_config_t line_pid_cfg = {
    .line_sensor_pins = {
        LINE_SENSOR_L2_PIN,
        LINE_SENSOR_L1_PIN,
        LINE_SENSOR_C_PIN,
        LINE_SENSOR_R1_PIN,
        LINE_SENSOR_R2_PIN,
    },
    .sensor_active_low = true,
    .invert_steering = false,
    .kp = 320.0f,
    .ki = 35.0f,
    .kd = 25.0f,
    .kff = 180.0f,
    .dt_s = 1.0f / (float)CONTROL_HZ,
    .integral_limit = 1.2f,
    .base_speed_cmd = BASE_SPEED_CMD,
    .max_motor_cmd = MAX_MOTOR_CMD,
};

static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_22, .in2 = GPIO_NUM_23},
    .right = { .in1 = GPIO_NUM_20, .in2 = GPIO_NUM_21},

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
    if (pid_siguelineas_init(&line_pid_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar pid_siguelineas");
        motor_mcpwm_stop(&motors);
        vTaskDelete(NULL);
    }

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t log_count = 0;

    while(1) {
        float curvature_ff = 0.0f;
        uint32_t curvature_ts_ms = 0;
        (void)shared_memory_get_curvature_ff(&curvature_ff, &curvature_ts_ms, 0);

        int16_t left_cmd = 0;
        int16_t right_cmd = 0;
        pid_siguelineas_debug_t dbg = {0};
        if (pid_siguelineas_step(curvature_ff, &left_cmd, &right_cmd, &dbg) == ESP_OK) {
            motor_mcpwm_set(&motors, left_cmd, right_cmd);
        } else {
            motor_mcpwm_stop(&motors);
        }

        shared_memory_heartbeat_cpu0();
        if ((log_count++ % CONTROL_HZ) == 0) {
            ESP_LOGI(TAG,
                     "PID linea err=%.3f ff=%.3f u=%.1f cmd=(%d,%d) line=%d ts=%lu",
                     dbg.line_error,
                     dbg.curvature_ff,
                     dbg.control_u,
                     dbg.left_cmd,
                     dbg.right_cmd,
                     dbg.line_detected ? 1 : 0,
                     (unsigned long)curvature_ts_ms);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
