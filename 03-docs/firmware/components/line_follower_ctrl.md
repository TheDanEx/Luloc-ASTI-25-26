# Componente: Line Follower Controller (`line_follower_ctrl`)

## Propósito Arquitectónico
Este componente aísla puramente la lógica matemática y algorítmica de seguimiento de línea (PID y Feed-Forward predictivo mediante cámara) del acceso al hardware. Su objetivo es recibir datos brutos del estado físico del robot (posición de la línea, pérdida total de línea, curvatura predicha por la cámara), calcular un lazo de control PID para mantener la pista y escupir la velocidad requerida para cada motor diferencial. Además, incorpora heurística para superar bifurcaciones y bloqueos, como los vértices de pistas en "rombo", tendiendo la trayectoria a ir "recta" si la línea central se pierde pero se iba centrado previamente.

## Entorno y Dependencias
- **Framework:** ESP-IDF v5.5.
- **Hardware:** Agnostico (Solo recibe floats).
- **Dependencias Locales:** Puesta a punto con `esp_log.h` y `esp_err.h`.
- **Integración:** Diseñado para ejecutarse desde una tarea FreeRTOS de alta frecuencia (p.ej., `task_rtcontrol`) asegurando que el parámetro `delta_time_s` representa exactamente el lapso de tiempo métrico transcurrido.

## Interfaces de E/S (Inputs/Outputs)
### Inputs:
- `line_follower_config_t`: Parámetros Kp, Ki, Kd, velocidades base y máximas, y `camera_weight` (multiplicador para el feed-forward).
- `line_follower_input_t`: Struct que se inyecta por ciclo:
  - `line_position` [-1.0 a 1.0]: Posición de la línea detectada.
  - `line_detected` [bool]: Flag indicando si al menos algún sensor analógico detecta línea.
  - `camera_curvature` [-1.0 a 1.0]: Estimación de la curvatura de la curva que viene.

### Outputs:
- `line_follower_output_t`: Struct de salida con velocidades deseadas:
  - `left_motor_speed`
  - `right_motor_speed`

## Flujo de Ejecución Lógico
1. El usuario inyecta el `input` y un `delta_time_s` en la función `line_follower_ctrl_update`.
2. **Evaluación de Pérdida de Línea:** Si `line_detected` es falso, revisa el `last_known_position`.
   - Si se perdió a un extremo (< -0.4 o > 0.4), ordena giros bruscos forzados para recuperar la línea (ruedas en sentido inverso).
   - Si se perdió en el centro (-0.4 a 0.4), intuye que es un cruce, diamante o brecha en la pista, e instruye al robot a mantener la velocidad base (ir recto) asistido por el Feed-Forward de la cámara si esta disponible.
3. **Lazo PID Base:** Si se detecta línea, computa el error en base a la `line_position` multiplicada por las constantes, aplicando la integral multiplicada por `delta_time_s`.
4. **Acoplamiento de Feed-Forward:** Añade el factor predictivo multiplicativo de la cámara mediante `camera_weight * camera_curvature`.
5. **Comando Diferencial:** Asigna la velocidad resultando a los motores restando/sumando a la `base_speed`.
6. **Limitación (Clamp):** Recorta estrictamente las salidas para no exceder `max_speed` y retorna.

## Funciones Principales y Parámetros
- `esp_err_t line_follower_ctrl_create(...)`: Inicializa la estructura interna, reseteando la integral a cero.
- `esp_err_t line_follower_ctrl_update(handle, input, output, delta_time_s)`: Función principal ejecutada en tiempo real.
- `esp_err_t line_follower_ctrl_set_pid(...)` y `_set_camera_weight`: Permite la actualización en caliente de los parámetros, típicamente llamado al recibir tramas MQTT.

## Puntos Críticos y Depuración
- **Windup Integral:** La integral se resetea automáticamente si se pierde la línea, lo que previene que el robot salga disparado hacia un lado al reencontrarla. Si aun así hay oscilaciones bruscas al salir de curvas cerradas, reducir inmediatamente `Ki`.
- **Feed-Forward Competidor:** Si `camera_weight` es excesivo, la dirección impuesta por la cámara forzará la salida de la línea recta antes de llegar a la curva, desencadenando la lógica de pérdida de línea prematuramente.
- **División por Cero:** El código cuenta de forma hard-coded un resguardo `if (delta_time_s <= 0.0f) delta_time_s = 0.01f;` para evitar excepciones del microprocesador en loops RTOS saturados.

## Ejemplo de Uso e Instanciación
```c
#include "line_follower_ctrl.h"

// 1. Configuracion en arranque de sistema (app_main o inicializacion)
line_follower_config_t config = {
    .kp = 1.8f, .ki = 0.01f, .kd = 0.5f,
    .base_speed = 40.0f,
    .max_speed = 100.0f,
    .camera_weight = 15.0f // Multiplicador para curva
};

line_follower_ctrl_handle_t ctrl_handle;
ESP_ERROR_CHECK(line_follower_ctrl_create(&config, &ctrl_handle));

// 2. Uso dentro del lazo cerrado de control (ej. 100 Hz / cada 10ms)
void task_rtcontrol(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float dt = 0.01f; // 10ms

    while(1) {
        // Obtenemos los datos fisicos (ejemplo pseudo-codigo)
        line_follower_input_t input = {
            .line_position = get_line_sensor_pos(), // ej. 0.2
            .line_detected = is_line_detected(),    // bool
            .camera_curvature = get_mqtt_cam_curv() // ej. -0.5
        };

        line_follower_output_t speeds;

        // Calculamos siguiente frame
        line_follower_ctrl_update(ctrl_handle, &input, &speeds, dt);

        // Actuamos en hardware
        // motors_set_speed(speeds.left_motor_speed, speeds.right_motor_speed);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
```
