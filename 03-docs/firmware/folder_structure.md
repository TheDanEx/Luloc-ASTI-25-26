# Folder Structure (Estructura del Proyecto de Firmware)

> **Nota de Nomenclatura:** La arquitectura de directorios de este proyecto requiere que **todos los nombres de carpetas y archivos estén en inglés** (Ej: `components/`, `folder_structure.md`). El contenido explicativo de los archivos puede estar en español o inglés libremente.

El firmware obedece a una distribución estricta promovida por el framework de ESP-IDF para garantizar modularidad y la compilación paralela de código embebido aislado.

## Directorios Fundamentales

### `01-firmware/ESP32-P4-ETH/main/`
Cerebro de la orquestación RTOS y configuración central.
- **`app_main.c/system_init.c`**: Puntos de ignición y configuración agresiva del Hardware (Logs, NVS, Sonido). 
- **`tasks/`**: Contiene la definición y el bucle interno aislado (`while(1)`) de los diferentes hilos (Cores) de ejecución (`task_comms_cpu1`, `task_rtcontrol_cpu0`). Aquí ocurre la orquestación asíncrona pero jamás las funciones lógicas o matemáticas densas.

### `01-firmware/ESP32-P4-ETH/components/`
Librerías estáticas aisladas (Independientes por diseño). Permite desarrollar módulos sin afectar a los demás. Tienen su propio `CMakeLists.txt`.
- **`motor/`**: Abstracción y control PIDs del hardware MCPWM del ESP32-P4, agnóstico al FreeRTOS externo.
- **`encoder_sensor/`**: Controlador hardware (pulse_cnt) de los motores.
- **`telemetry_manager/`**: Módulo Backend que procesa la agrupación asíncrona de datos para la generación del Influx Line Protocol (ILP Batching).
- **`mqtt_custom_client/`**: Envoltorio de LwIP dedicado puramente al hilo de las comunicaciones IoT y subscripciones.
- **`ptp_client/`**: Gestión explícita de alto nivel para IEEE 1588 sincronizado con el Master (Docker RPi5), expone el timestamp unificado y corregido en microsegundos.

### Otros Archivos de Relevancia
- **`sdkconfig` / `sdkconfig.defaults`**: El ADN del firmware. Contiene los _toggles_ generados por el comando `menuconfig` para preprocesadores de C. Controla stacks TCP/IP, IGMP, Multicast PTP nativo del ESP32, e información binaria de Flash (Ej: Ocurrencia de SW/HW Timestamping o configuración general FreeRTOS).
