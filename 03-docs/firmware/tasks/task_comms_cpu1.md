# Task Comms (task_comms_cpu1)

## Propósito Arquitectónico
Hilo maestro de comunicaciones asíncronas enfocado en la interfaz humano-máquina y la nube (Networking/Telemetry). Libera de la carga TCP/IP y de formateo String/JSON a la rutina de control de vuelo, corriendo en su propio procesador de manera independiente.

## Entorno y Dependencias
- FreeRTOS Task anclada al **Core 1** del ESP32 con **Prioridad 10**.
- Dependiente fuertemente del stack de red (LwIP), el `mqtt_custom_client` y librerías de `telemetry_manager`. Coordina subsistemas base de monitoreo como `encoder_sensor` y `perf_mon`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Lectura directa y exclusiva de los pines del Encoder (GPIO 18 y 19).
- **Software:** 
  - Produce hacia `robot/odometry` y `robot/telemetry` a 1Hz. 
  - Consume comandos entrantes desde `robot/cmd`. 
  - Abre y gestiona una **Inter-core Queue** (Cola entre procesadores) para volcar peticiones sin ensuciar memoria cruda.

## Flujo de Ejecución Lógico
Bloque 1 inicializador secuencial (NVS, Red, Tópicos, Encoders). Tras el setup, entra en el típico macro-bucle infinito (`while(1)`).
Ciclo ultra-rápido de 10ms (`POLL_INTERVAL_MS`): extrae inmediatamente comandos asíncronos y los despacha mediante el parser string. 
Ciclo temporizado lento (1000ms): `collect_sensor_data()` barre los encoders, calcula el uptime y ordena a `telemetry_manager` que emita el paquete MQTT compuesto.

## Funciones Principales y Parámetros
- `task_comms_cpu1_start(void)`: Punto de entrada (Spawn) lanzado comúnmente desde `app_main`. Aloja la pila principal (8192 bytes) y amarra la función estática al CPU1.
- `task_comms_cpu1_init_queue(void)`: Crea dinámicamente el `command_queue` de 32 posiciones de 256 bytes cada una.
- `task_comms_cpu1_get_queue(void)`: Devuelve el identificador global genérico de la cola (`QueueHandle_t`) para que tareas ajenas (como interrupciones seriales u otros callbacks) inyecten trabajo al core 1 sin bloqueo duro.
- `task_comms_cpu1_is_ready(void)`: Booleano para alertar a otros si el MQTT reaccionario ha sido inicializado limpiamente en red.

## Puntos Críticos y Depuración
- **Stack Overflow (Desbordamiento de Pila):** Las librerías JSON, librerías MQTT y formateos (`snprintf`) consumen un stack agresivo. Si se añaden más tópicos anidados, la tarea explotará. Observar Watermarks con Perf Mon.
- **Queue Blocking:** Si el parser lógico (`handle_mqtt_command`) empieza a hacer delays largos u operaciones bloqueantes de red en crudo (esperar un ACK del HTTP Influx por ejemplo), la cola inter-core se saturará (`CMD_QUEUE_SIZE 32`) perdiendo eventos o telemandos.
