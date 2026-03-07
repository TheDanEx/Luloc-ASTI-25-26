# Buenas Prácticas de Desarrollo - Lurloc-ASTI

Este documento establece las normativas estrictas de desarrollo que rigen el ecosistema de Lurloc-ASTI, tanto para el firmware (ESP32) como para la capa de software (Docker / SBC). Todo código contribuido debe pasar por el filtro de estas guías.

## 1. Prioridad: Legibilidad y Estructura
El código se escribe una vez, pero se lee mil veces.
- **Evite el código denso o sobre-optimizado prematuramente:** A menos que un lazo PID o procesamiento de visión lo exija matemáticamente, opte por construcciones lógicas legibles sobre los "one-liners".
- **Código Auto-Documentado (Minimismo de Comentarios):** El mejor comentario es un buen código. Evite redundancias lógicas (ej. no comente `// Incrementa variable`). Utilice nombres de variables y funciones suficientemente descriptivos para que la lógica se explique sola.
- **Uso Estricto y Breve de Comentarios:** LOS COMENTARIOS DEBEN SER POCOS. Únicamente se validarán explicaciones *breves* de 1-2 líneas en bloques altamente complejos generados por IA (ej. matemáticas de cuaterniones, hacks de registros físicos) para explicar el *por qué* de la decisión algorítmica. Jamás sobre-comente el flujo evidente.

## 2. Nomenclatura (Naming Conventions)
- **Idioma del Proyecto:** TODO archivo, carpeta secundaria, módulo, variable, y función DEBE estar nombrado enteramente en **Inglés**. A la vez, el contenido de la documentación (los Párrafos) puede redactarse en el idioma del equipo para mayor fluidez, pero la topología del árbol de archivos es intocable y universal (Ej. `components/`, `folder_structure`, no `componentes/` ni `estructura`).
- **Commits en Git:** Los mensajes de los commits DEBEN estar escritos obligatoriamente en **Inglés**, siguiendo el formato convencional (ej. `feat: add PTP client`, `fix: memory leak in odometry`).
- **Funciones C/C++:** `snake_case`. Módulos explícitos (ej. `motor_mcpwm_set` en vez de `setMotor`).
- **Variables de Estado:** Deben insinuar su uso (ej. `is_mqtt_connected`, `last_odometry_update_ms`).
- **Macros y Constantes:** `UPPER_SNAKE_CASE` (ej. `CMD_QUEUE_SIZE_MAX`).

## 3. Mantenimiento Vivo de la Documentación
- **Sincronización Código-Docs:** CUALQUIER cambio (refactorización, adición de parámetros, nuevos módulos, o modificaciones de puertos/entornos) en el Firmware o el Software **obliga** al desarrollador o IA a actualizar inmediata y silenciosamente los archivos correspondientes en la carpeta `03-docs/`. Un Pull Request o Commit  se considera inválido si el código cambió pero su documentación asociada quedó obsoleta.

## 4. Manejo de Errores y Excepciones
- **En Firmware (ESP-IDF):** Jamás ignore resultados de APIs. Utilice `ESP_ERROR_CHECK()` sólo durante la inicialización (`system_init` o boot de hilos). Durante ejecución normal (launches iterativos) capture `esp_err_t` y gestione *gracefully* (por ejemplo, publicando un fallo a Logger Central o Telemetría) sin reiniciar el chip abruptamente.
- **En Software (ROS2/Python):** Tracebacks deben ir acompañados de un log explicativo (ros_logger info/error) antes de capturar el try/except, nunca capture `Exception` genérico sin relanzarlo o actuar al respecto.

## 5. Estructura y Modularidad Espacial
- **Separación de Tareas (RTOS):** No instancie librerías bloqueantes ajenas al flujo real-time dentro de controladores de interrupción (ISR) o bucles de Prioridad 10. Delega por Colas (`xQueueSendFromISR`).
- **Encapsulación Docker:** Cada nodo en Docker debe ser inmutable y su configuración debe depender estrictamente de variables extraídas del `.env`. No hardcodee URIs locales de la SBC.

## 6. Pruebas y Validación Unitaria
- Cualquier nuevo bloque lógico (PIDs analíticos, procesadores JSON) debe ofrecer una función inactiva de AutoTest en frío que pueda ejecutarse desde un entorno virtual sin la necesidad de montar los periféricos ESP32 en el robot real.

## 7. Estándar de Telemetría Estructurada (MQTT Batching / Influx Line Protocol)
Para evitar la saturación de TCP/IP, Mosquitto y el Host (SBC) por ráfagas de datos de alta frecuencia (Ej. control PID a 1KHz), TODO módulo que envíe datos telemétricos o variables observadas al Broker MQTT bajo la rama `robot/telemetry/*` u `odometry/*` DEBE implementarse siguiendo este formato estricto:

### A. Protocolo de Formato de Salida
El formato final transmitido al Broker MQTT y procesado por Telegraf no será un JSON clásico por cada lectura escalar, sino un bloque comprimido basado o adaptado al **Influx Line Protocol (ILP)**, que permite ingestón sub-milisegundo nativa en InfluxDB y ploteo con precisión extrema en Grafana.

Ejemplo del formato de salida transmitido por MQTT (Varias líneas empaquetadas en un solo Payload):
```text
odometry velIZ=1.23,posIZ=0.5 1677628800000000
odometry velIZ=1.24,posIZ=0.8 1677628800001000
odometry velIZ=1.25,posIZ=1.1 1677628800002000
```
- `<Measurement>` (Ej: `odometry`): Define la tabla general.
- `<Fields>` (Ej: `velIZ=1.23,posIZ=0.5`): Conjunto clave-valor separados por comas.
- `<Timestamp>` (Ej: `1677628800001000`): **OBLIGATORIO en precisión de microsegundos (`esp_timer_get_time()`)** separado de los fields por un espacio.  El reloj del ESP32 debe adjuntarlo en cada lectura al vuelo. Grafana usará esta estampa de tiempo exacta (Absolute Time) en lugar del tiempo en que el dato llegó al servidor.

### B. Ciclo de Vida del Transmisor (Batching en SRAM)
La recolección de alta frecuencia no modifica las llamadas de la API superior de los componentes, pero `telemetry_manager.c` por debajo (el backend) no publicará en MQTT Inmediatamente:
1. **Acumulación (High Frequency):** La rutina de control rápido llamará a `telemetry_add_float()` múltiples veces por milisegundo o decasegundo en el buffer privado. El manejador apilará líneas de string internamente y les adosará el microsegundo exacto de hardware del instante en el que recivió la petición.
2. **Ventana de Envío (Publish Interval de Baja Frecuencia):** Únicamente cuando el Temporizador del manejador de telemetría venza (`interval_ms`, usualmente entre `500ms` a `2000ms`) o cuando el Buffer SRAM pre-asignado roce su capacidad máxima (Max Chunk Size), se cerrcerá el Payload concatenado y se lanzará un ÚNICO `mqtt_custom_client_publish()`.
3. Esto mantiene inalterada la usabilidad para los desarrolladores (Simples llamadas a `add_float`) pero previene que un bucle `while_1` saturado cause un DoS al Broker.
