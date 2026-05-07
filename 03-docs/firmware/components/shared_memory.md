# Shared Memory (IPC Manager)

## Propósito Arquitectónico
Este componente actúa como el puente de comunicación inter-procesos (IPC) entre el **Core 0 (Comunicaciones/uROS)** y el **Core 1 (Control en Tiempo Real)**. Su arquitectura está diseñada para garantizar que el Core 1 nunca se bloquee, manteniendo el determinismo del lazo de control de 100Hz, mientras que el Core 0 puede consumir la información más reciente de forma asíncrona.

## Entorno y Dependencias
- **FreeRTOS Queues:** Utiliza `xQueueOverwrite` para flujos de datos continuos y colas estándar para comandos.
- **ESP-IDF:** Compatible con la arquitectura multi-core del ESP32-P4.

## Interfaces de E/S (Inputs/Outputs)
- **Entradas (desde Core 1):** Telemetría (sensores, PIDs) y Estado (batería, modo).
- **Entradas (desde Core 0):** Comandos de movimiento (Twist) y Comandos de modo.
- **Salidas:** Provee una API opaca para extraer estos datos desde el núcleo opuesto.

## Flujo de Ejecución Lógico
1. **Productor Rápido -> Consumidor Lento (Core 1 a Core 0):** Se utiliza `xQueueOverwrite` con tamaño 1. El Core 1 escribe constantemente; el Core 0 lee cuando está listo, obteniendo siempre la muestra más fresca.
2. **Productor Lento -> Consumidor Rápido (Core 0 a Core 1):** Se utilizan colas FIFO. El Core 1 realiza un *polling* no bloqueante (timeout 0) para procesar comandos en cuanto llegan sin detener su ciclo de ejecución.

## Funciones Principales y Parámetros
- `shared_memory_init()`: Inicializa todas las colas internas.
- `shared_memory_push_telemetry(const robot_telemetry_t *data)`: Envía datos desde el Core 1.
- `shared_memory_get_telemetry(robot_telemetry_t *data)`: Recupera datos desde el Core 0.
- `shared_memory_push_twist_cmd(const robot_twist_cmd_t *cmd)`: Envía comandos desde el Core 0.
- `shared_memory_get_twist_cmd(robot_twist_cmd_t *cmd)`: Recupera comandos desde el Core 1.

## Puntos Críticos y Depuración
- **No Bloqueo:** El Core 1 **JAMÁS** debe llamar a funciones de lectura/escritura con un timeout distinto de 0.
- **Pérdida de Comandos:** Si se envían múltiples comandos de modo muy rápido desde ROS, la cola (tamaño 5) podría llenarse. Se asume que los cambios de modo son eventos discretos lentos.

## Ejemplo de Uso e Instanciación
```c
#include "shared_memory.h"

// En Core 1 (Control)
void control_loop() {
    robot_telemetry_t telem = { ... };
    shared_memory_push_telemetry(&telem); // Non-blocking

    robot_twist_cmd_t cmd;
    if (shared_memory_get_twist_cmd(&cmd) == ESP_OK) {
        // Aplicar velocidad
    }
}
```
