# Shared Memory

## Propósito Arquitectónico
Provee el mecanismo de acoplamiento débil principal del sistema entero para Inter-Process y Multi-Core Communication (IPC). Elimina condiciones de carrera asegurándose de que una CPU de control (CPU0) pura y agresiva puede enviar datos al mundo exterior operado por una CPU de comunicaciones (CPU1) y viceversa usando simples estructuras comunes.

## Entorno y Dependencias
Core Semaphores primitives de FreeRTOS (`freertos/semphr.h`). 

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Todo se encuentra anclado a buffers en SRAM local compartida.
- **Software:** Wrapper sobre una gran directiva estática `shared_memory_t` separando sub-bloques de Sensores (`robot_sensor_data_t`) y Comandos (`robot_command_t`). Usa mutadores y descriptores con Timeout estricto de concurrencia (`shared_memory_read/write_sensors()`). Implementa variables bidireccionales de 'Heartbeat' para chequeos pasivos de vida y estados MQTT (`mqtt_connected`).

## Flujo de Ejecución Lógico
Instanciación única en inicio (`shared_memory_init()`). Cada bloque (Control o Comms) llama a primitivas `write/read` con un `TickType_t` max timeout. Internamente, un Mutex asegura que los structs de bytes múltiples jamás se crucen a la mitad (Lectura sucia). Un núcleo también escribe periodicamente incrementando un `heartbeat_cpuX++` y valida el contiguo. 

## Funciones Principales y Parámetros
- `shared_memory_init(void)`: Pide el Mutex inicial de RTOS y blanquea el struct global para que ambas CPUs puedan empezar.
- `shared_memory_write_sensors(const robot_sensor_data_t *data, TickType_t timeout)`: (Uso típico CPU0) Transmite lo leído del hardware al pool central.
  - `data`: Puntero al struct local rellenado de lecturas de hardware.
  - `timeout`: Bloqueo máximo dispuesto a esperar por el mutex (ej. `pdMS_TO_TICKS(5)`).
- `shared_memory_read_sensors(robot_sensor_data_t *data, TickType_t timeout)`: (Uso típico CPU1) Clona lo almacenado para subirlo a telemetría.
- `shared_memory_write_command(...)` / `_read_command(...)`: Flujo inverso para las directivas desde la red (CPU1) a los motores (CPU0).
- `shared_memory_heartbeat_cpu0(void)` y `_cpu1(void)`: Incrementan contadores internos para verificar que el núcleo contrario no se bloqueó en un bucle infinito (Watchdog de software mutuo).

## Puntos Críticos y Depuración
- **Deadlocks / Fallo asimétrico:** Si el Timeout de bloqueo fuera infinito (por mala implementación de otra capa o bugs de Mutex), la CPU completa en modo Real-Time quedaría sentenciada al paro incondicional. Asegurar ticks de timeout máximos no superen los ciclos de control esperados (e.g. max 2-5ms).
- **Lectura Cruda de Puntero Peligrosa:** Ofrece acceso profundo directo a `shared_memory_get()`. Quien acceda y asigne punteros directos sin tomar el lock corromperá temporalmente los arreglos en arquitecturas SMP en momento de contención extrema.
