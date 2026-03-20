# State Machine

## Propósito Arquitectónico
Motor semántico que dicta los sub-estados de permisos, autonomía y salvaguardas reactivas para la tarea en tiempo real del robot (CPU0). Protege módulos motores cortando flujos si ocurre un cambio no permitido o pérdida del estado MQTT si la máquina fue definida para requerirlo.

## Entorno y Dependencias
Dependencia conceptual directa contra tipos pre-formados (`robot_mode_t`, `robot_state_t` descritos en cabeceras de configuración). Interactúa por debajo con la Memoria Compartida (`Shared Memory`) para percibir el estado de MQTT (`state_machine_notify_mqtt_status`).

## Interfaces de E/S (Inputs/Outputs)
- **Software:** Consultas continuas sobre el modo (`state_machine_update()`) que computan y retornan al llamador el estado evaluado actual de la misma máquina. Se admiten requests asíncronos (`state_machine_request_mode()`). Ofrece validadores directos (`state_machine_is_autonomous_safe()`).

## Flujo de Ejecución Lógico
Evaluación continua. Cada bucle maestro llama a `update()`. Internamente si "AUTONOMOUS" estricto es activado, se asume ciego pero estable. Si cambia a estado "REMOTE_CONTROLLED" se requiere confirmación permanente de que `mqtt_connected == true`. Si se reporta caída, la máquina de estados hace rollback en caliente a estado "ERROR", denegando todos los flags de set-points (aparentemente frenos). Se miden estancias absolutas de ms ("state_time_ms") para timeouts.

## Funciones Principales y Parámetros
- `state_machine_init(void)`: Restablece variables iniciales marcando el estado nativo de sistema como INIT.
- `state_machine_update(void)`: Calcula la permanencia temporal y evalúa si las condiciones presentes provocan una transición (como red caída). Retorna el enum de `robot_state_t` más reciente.
- `state_machine_notify_mqtt_status(bool connected)`: Inyecta el flag booleano indicando si el cliente remoto está enlazado.
- `state_machine_request_mode(robot_mode_t new_mode)`: Solicita cambiar a un modo funcional (`AUTONOMOUS`, `REMOTE_CONTROLLED`).
  - `new_mode`: Enum indicando la meta deseada. Retorna false si el paso no es permitido por salvaguardas perimetrales de seguridad.
- `state_machine_is_autonomous_safe(void)`: Retorna un booleano puro que sirve de validación ultrarrápida (Lock condicional) en un Task RTOS antes de encender un lazo PID de ruedas ciego.

## Puntos Críticos y Depuración
- **Loop Irrecuperable en "ERROR":** Transición abrupta forzada si se pierde la red en pleno movimiento. Depurar esto es vital: al reactivarse el MQTT no debe saltar disparado (el "Heartbeat / Timeout" de comandos ha de caducar antes u ocasionará accidentes por comandos retenidos / retransmitidos en rediseños). No requiere intervenciones pero debe ser bien documentado en frontends.

## Ejemplo de Uso e Instanciación
```c
#include "state_machine.h"

// 1. Inicialización en el arranque temprano
void system_boot(void) {
    state_machine_init(); // Set a estado INIT y relojes a 0
}

// 2. Evento red - Desde la Tarea MQTT o Ethernet (Core 1)
void on_mqtt_connect(void) {
    state_machine_notify_mqtt_status(true);
    // Solicitamos entrar en modo teleoperado
    state_machine_request_mode(MODE_REMOTE_CONTROLLED);
}

// 3. Bucle de Control PID (Core 0)
void pid_task(void *pvParameters) {
    while(1) {
        // Cómputo constante de transiciones y Timeouts
        robot_state_t actual = state_machine_update();

        // Validar seguridad perimetral antes de activar PWM Motores
        if (actual == STATE_ERROR) {
            motor_mcpwm_brake(drv_motores); // Fallo de red, detención absoluta
        } else if (actual == STATE_REMOTE_CONTROLLED || state_machine_is_autonomous_safe()) {
            // El robot tiene permitida la movilidad.
            execute_pid_loop();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```
