# Logger

## Propósito Arquitectónico
Provee un mecanismo central y seguro entre hilos para la emisión de cadenas de texto y telemetría de fallos del sistema sin bloquear los hilos (Time-Critical CPU cores). 

## Entorno y Dependencias
Core de FreeRTOS (`freertos/queue.h`), interactúa seguramente por encima del `esp_log.h` si busca aislar concurrencia.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** UART física principal empleada como salida a la terminal Serial, o memoria flash para dumps persistentes.
- **Software:** `logger_send(level, fmt, ...)` reemplaza a iteraciones normales de subida directa. Niveles fijos: INFO, WARN, ERROR.

## Flujo de Ejecución Lógico
Recibe flujos de texto asíncronos encolando cadenas dinámicas o punteros en una cola FreeRTOS, luego una tarea de baja prioridad extrae las tramas y vacía sobre la terminal (serial o remota).

## Funciones Principales y Parámetros
- `logger_init(void)`: Inicializa y levanta explícitamente la cola subyacente de RTOS (Queue) y la tarea consumidora en segundo plano que despachará los mensajes asíncronamente por serial.
- `logger_send(log_level_t level, const char* fmt, ...)`: Versión asíncrona, embebida y Thread-Safe de `printf`. Encola sin bloquear.
  - `level`: Enum de severidad del mensaje (`LOG_INFO`, `LOG_WARN`, `LOG_ERROR`).
  - `fmt`: Argumento obligatorio de cadena de texto base con formato (estilo printf clásico de C, ej: `"Voltage: %.2f mV"`).
  - `...`: (Variable Arguments) Argumentos variádicos correspondientes a los especificadores de formato indicados en `fmt`.

## Puntos Críticos y Depuración
- **Saturación y Desbordes:** Si la cola se llena demasiado rápido (modo debug extremo de motores), las cadenas se perderán silenciosamente afectando la depuración real.
- **Inversión de prioridades / Allocation:** Ocurre si internamente se realiza `malloc` o formato en el hilo productivo y no en el hilo consumidor, perjudicando gravemente la latencia de las tareas de control RT.

## Ejemplo de Uso e Instanciación
```c
#include "logger.h"

// 1. En el hilo de inicio más bajo (ej. system_init.c)
void init_base_services(void) {
    // Aloca la memoria de la cola y crea el Task de Logging (Core 0, Baja prioridad)
    logger_init();
    
    // Primer log disparado de forma no bloqueante
    logger_send(LOG_INFO, "Servicio de Logging asíncrono inicializado. Core Libre.");
}

// 2. Desde cualquier otra tarea, por crítica que sea (ej. Control PID a 1000Hz)
void pid_control_task(void *pvParameters) {
    while(1) {
        float tension = get_motor_voltage();
        if(tension > 24.5f) {
            // Este log NO bloquea el PID. Se delega formato variádico a la cola RTOS
            logger_send(LOG_ERROR, "¡Sobretensión detectada! V=%.2f. Cortando...", tension);
            emergency_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```
