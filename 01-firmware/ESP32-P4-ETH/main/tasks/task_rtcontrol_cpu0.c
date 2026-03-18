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

#include "telemetry_manager.h"

static const char *TAG = "rt_cntrl";

// Motor Configuration (PINS: Iz: 47/48, Dr: 20/21)
static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_22, .in2 = GPIO_NUM_23},
    .right = { .in1 = GPIO_NUM_21, .in2 = GPIO_NUM_20},

    .nsleep = GPIO_NUM_NC,
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
    motor_velocity_config_t cfg_l = {
        .kp              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KP),
        .ki              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KI),
        .kd              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KD),
        .max_battery_mv  = atof(CONFIG_VELOCITY_CTRL_MAX_BATTERY_MV),
        .max_motor_speed = atof(CONFIG_VELOCITY_CTRL_MAX_MOTOR_SPEED_MS),
        .deadband_v      = atof(CONFIG_VEL_CTRL_DEADBAND_V),
        .accel_limit_ms2 = atof(CONFIG_VEL_CTRL_ACCEL_LIMIT),
        .ema_alpha       = atof(CONFIG_VEL_CTRL_EMA_ALPHA)
    };
    motor_velocity_config_t cfg_r = cfg_l;

    pid_tuner_load_motor_pid(0, &cfg_l.kp, &cfg_l.ki, &cfg_l.kd);
    pid_tuner_load_motor_pid(1, &cfg_r.kp, &cfg_r.ki, &cfg_r.kd);

    motor_velocity_ctrl_handle_t ctrl_left, ctrl_right;
    motor_velocity_ctrl_create(&cfg_l, &ctrl_left);
    motor_velocity_ctrl_create(&cfg_r, &ctrl_right);

    const float dt = 0.01f; // 10ms = 100 Hz
    const TickType_t poll_rate = pdMS_TO_TICKS(10); 

    while(1) {
        robot_state_context_t* ctx = state_machine_get_context();
        shared_memory_t* shm = shared_memory_get();

        xSemaphoreTake(shm->mutex, portMAX_DELAY);

        // Update PID live tuning si hay cambios desde MQTT
        for (int i = 0; i < 2; i++) {
            if (shm->motor_pids[i].updated_flag) {
                motor_velocity_ctrl_set_pid((i == 0) ? ctrl_left : ctrl_right, 
                                          shm->motor_pids[i].kp, 
                                          shm->motor_pids[i].ki, 
                                          shm->motor_pids[i].kd);
                shm->motor_pids[i].updated_flag = false;
            }
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
