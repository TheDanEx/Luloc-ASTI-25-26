# Componente: Motor Velocity Controller (`motor_velocity_ctrl`)

## Propósito Arquitectónico
Este componente actúa como una capa de abstracción lógica (middleware matemático) entre los generadores de trayectorias (como el `line_follower_ctrl` o un Gamepad) y los transistores de bajo nivel de los motores.
El problema común en robótica es que un PWM fijo (ej. 50%) produce velocidades drásticamente distintas si la batería LiPo está recién cargada a 16.8V frente a cuando está cascada a 14V. Este módulo resuelve este problema implementando un control **Closed-Loop (PID)** mezclado con un **Open-Loop Feed-Forward basado en voltaje**.
Recibe comandos en forma de `target_speed` (una métrica física universal en m/s) y deduce el `PWM` instantáneo necesario compensando la descarga en tiempo real capturada desde la memoria compartida por el `ina219_sensor`.

## Entorno y Dependencias
- **Framework:** ESP-IDF v5.5.
- **Hardware:** Agnostico. No invoca periféricos directamente, sólo procesa floats.
- **Dependencias Locales:** Puesta a punto con `esp_log.h`.

## Interfaces de E/S (Inputs/Outputs)
### Configuración (`motor_velocity_config_t`):
- `kp, ki, kd`: Ganancias PID para corregir el error dinámico de la rueda.
- `max_battery_mv`: Voltaje de referencia ideal de la 4S LiPo (p. ej., 16800.0 mV).
- `max_motor_speed`: M/s (metros por segundo) físicos que alcanza el motor cuando recibe `max_battery_mv` a 100% de PWM.

### Inputs por Ciclo (`motor_velocity_input_t`):
- `target_speed`: Velocidad deseada (m/s) dictada por la Inteligencia Artificial o el Siguelíneas.
- `current_speed`: Velocidad real (m/s) devuelta por el sensor `encoder_sensor`.
- `battery_mv`: Voltaje instantáneo de la batería leído por el INA219.

### Output:
- Float: Porcentaje PWM a inyectar al driver del motor (`-100.0` a `100.0`).

## Flujo de Ejecución Lógico
1. El usuario inicializa el controlador de motor (se debe utilizar **una instancia diferente para cada motor independiente**, por ejemplo, `left_vel_ctrl` y `right_vel_ctrl`).
2. En cada iteración del lazo de control RTOS (`task_rtcontrol_cpu0`), se inyectan la velocidad objetivo, actual y el voltaje de la batería en ese instante.
3. El controlador primero procesa un **Feed-Forward**:
   - Computa la fracción porcentual solicitada en relación a sus topes: `target_speed / max_motor_speed`.
   - Modifica este PWM teórico multiplicándolo por un **factor compensador de voltaje**: `max_battery_mv / battery_mv`. Si la batería cae a 14V (respecto al ideal de 16.8V), el factor es 1.2, forzando un PWM 20% superior por anticipado sin necesidad de que el PID acumule error de viento (Windup).
4. Luego ejecuta un lazo clásico **PID**:
   - Calcula el error `target_speed - current_speed`.
   - Modula y deriva, calculando un PWM de corrección diminuto.
5. El algoritmo suma el Feed-Forward Predictivo + La corrección PID Reactiva, lo recorta estrictamente entre `-100.0` y `100.0` y devuelve el valor deseado.

## Funciones Principales y Parámetros
- `esp_err_t motor_velocity_ctrl_create(...)`: Inicializa la estructura interna, reseteando las integrales a cero. Retorna una handle opaca `motor_velocity_ctrl_handle_t`.
- `esp_err_t motor_velocity_ctrl_update(...)`: Función principal que ejecuta la matemática por Frame de Control.

## Puntos Críticos y Depuración
- **Prevención de Divide-by-Zero:** Si el cable sensórico del INA219 se arranca en pleno movimiento, el `battery_mv` devuelto por fallback puede ser `0`. El algoritmo cuenta internamente de forma defensiva con un control seguro: si el voltaje del INA219 es irrisorio (< 5000mV en un robot de 12V), desecha la medición y usa estáticamente el `max_battery_mv`, comportándose como un PID básico ciego para no generar picos matemáticos monstruosos de PWM.
- **Dualidad de Contexto:** NUNCA recicle el *Handle* opaco para controlar dos motores. El componente depende de un `integral` y un `previous_error` propio que es único de la rueda izquierda, y otra Handle exclusiva para la rueda derecha.

## Ejemplo de Uso e Instanciación
```c
#include "motor_velocity_ctrl.h"

// 1. Instanciación en arranque de programa
motor_velocity_config_t m_config = {
    .kp = 0.5f, .ki = 0.05f, .kd = 0.01f,
    .max_battery_mv = 16800.0f, // 4S LiPo ideal
    .max_motor_speed = 1.5f     // m/s medidos al 100% PWM a 16.8V
};

motor_velocity_ctrl_handle_t ctrl_left, ctrl_right;
ESP_ERROR_CHECK(motor_velocity_ctrl_create(&m_config, &ctrl_left));
ESP_ERROR_CHECK(motor_velocity_ctrl_create(&m_config, &ctrl_right));

// 2. Uso dentro del lazo cerrado (ej. task_rtcontrol_cpu0)
void task_rtcontrol(void *pvParameters) {
    float dt = 0.01f;

    while(1) {
        // Pedimos velocidad lineal a la lógica superior
        float target_speed_l = 0.8f; // 0.8 metros/segundo

        // Leemos sensores y memoria (Pseudocódigo ilustrativo)
        shared_memory_t* shm = shared_memory_get();
        xSemaphoreTake(shm->mutex, portMAX_DELAY);
        float bat_mv = shm->sensors.battery_voltage;
        float actual_speed_l = encoder_sensor_get_speed(enc_left_handle); // Ya devuelve m/s
        xSemaphoreGive(shm->mutex);

        // Preparamos entrada
        motor_velocity_input_t input_l = {
            .target_speed = target_speed_l,
            .current_speed = actual_speed_l,
            .battery_mv = bat_mv
        };

        // Calculamos PWM compensado!
        float out_pwm_left;
        motor_velocity_ctrl_update(ctrl_left, &input_l, dt, &out_pwm_left);

        // Aplicamos PWM a los drivers lógicos del Motor H-Bridge
        // motors_set_left_speed(out_pwm_left); // -100 to 100

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```
