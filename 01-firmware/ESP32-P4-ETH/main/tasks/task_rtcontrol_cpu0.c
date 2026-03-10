#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"
#include "motor.h"
#include "state_machine.h"
#include "motor_velocity_ctrl.h"
#include "pid_tuner.h"
#include "shared_memory.h"
#include "mode_calibrate.h"
#include <stdlib.h>

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
    ESP_LOGI(TAG, "Control loop running");
    motor_mcpwm_init(&motors);
    
    // Configuración base leída de Kconfig (Cargada desde NVS si hay Live Tuning)
    motor_velocity_config_t m_config = {
        .kp = 0.5f, .ki = 0.05f, .kd = 0.01f, // Default, se sobreescribe
        .max_battery_mv = 16800.0f,
        .max_motor_speed = 1.5f
    };
    pid_tuner_load_motor_pid(&m_config.kp, &m_config.ki, &m_config.kd);

    motor_velocity_ctrl_handle_t ctrl_left, ctrl_right;
    motor_velocity_ctrl_create(&m_config, &ctrl_left);
    motor_velocity_ctrl_create(&m_config, &ctrl_right);

    const float dt = 0.01f; // 10ms = 100 Hz
    const TickType_t poll_rate = pdMS_TO_TICKS(10); 

    while(1) {
        robot_state_context_t* ctx = state_machine_get_context();
        shared_memory_t* shm = shared_memory_get();
        xSemaphoreTake(shm->mutex, portMAX_DELAY);

        // Update PID live tuning si hay cambios desde MQTT
        if (shm->live_pid.updated_flag) {
            motor_velocity_ctrl_set_pid(ctrl_left, shm->live_pid.kp, shm->live_pid.ki, shm->live_pid.kd);
            motor_velocity_ctrl_set_pid(ctrl_right, shm->live_pid.kp, shm->live_pid.ki, shm->live_pid.kd);
            shm->live_pid.updated_flag = false;
        }

        // Leer Telemetría local (si fuera necesario explícitamente aquí, o se delega al modo)
        
        xSemaphoreGive(shm->mutex);

        // --- ENRUTAMIENTO DE MODOS (Router Pattern) ---
        if (ctx->current_mode == MODE_CALIBRATE_MOTORS || ctx->current_mode == MODE_CALIBRATE_LINE) {
            
            // Delegamos la lógica al módulo de calibración dinámico
            mode_calibrate_execute(&motors, ctrl_left, ctrl_right, dt);
            
        } else {
            // Placeholder global: Por defecto, si el modo no está mapeado aún, frenamos.
            motor_mcpwm_stop(&motors);
        }

        vTaskDelay(poll_rate);
    }
}

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
