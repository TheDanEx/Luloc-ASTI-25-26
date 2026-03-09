# Luloc UJI Robotics Team

## Primera Memoria \- asti challenge 2026/27

##

# ÍNDICE {#índice}

[**ÍNDICE	1**](#índice)

[**Parte 0 \- Presentación del equipo	2**](#parte-0---presentación-del-equipo)

[**Parte 1 \- Diseño y desarrollo del robot	3**](#parte-1---diseño-y-desarrollo-del-robot)

[1\. Planificación y cronograma	3](#planificación-y-cronograma)

[2\. Proceso de diseño del robot	4](#proceso-de-diseño-del-robot)

[2.1. Diseño mecánico	4](#diseño-mecánico)

[2.2. Diseño eléctrico	4](#diseño-eléctrico)

[3\. Programación del robot	5](#programación-del-robot)

[3.1. Arquitectura	5](#arquitectura)

[3.2. Software	5](#software)

[3.3. Firmware	5](#firmware)

[4\. Montaje y construcción	6](#montaje-y-construcción)

[4.1. Impresión 3D	6](#impresión-3d)

[4.2. Conexiones eléctricas	6](#conexiones-eléctricas)

[4.3. Montaje mecánico	6](#montaje-mecánico)

[5\. Pruebas, validaciones y mejoras	7](#pruebas,-validaciones-y-mejoras)

[6\. Presupuesto	8](#presupuesto)

[7\. Financiación	9](#financiación)

[**Parte 2 \- Desafío "Automatización el futuro"	10**](#parte-2---desafío-"automatización-el-futuro")

[1\. Definición del proceso a mejorar	10](#definición-del-proceso-a-mejorar)

[2\. Descripción de la solución robótica propuesta	11](#descripción-de-la-solución-robótica-propuesta)

[3\. Modificaciones o adaptaciones al diseño original	12](#modificaciones-o-adaptaciones-al-diseño-original)

#

# Parte 0 \- Presentación del equipo {#parte-0---presentación-del-equipo}

Somos el equipo "Luloc UJI Robotics Team", formamos parte del equipo de robótica de la Universitat Jaume I. Este año hemos decidido participar con dos equipos formados mayoritariamente por alumnos de primer año de los grados de inteligencia robótica e ingeniería mecánica. Los dos equipos hemos trabajado en conjunto en las primeras fases del desarrollo, búsqueda de patrocinadores, presupuesto, y estructura del software y firmware.

Miembros del equipo:

**Daniel Ortiz Cormano \- Mentor**
**1º Inteligencia Robótica**
Con su [experiencia laboral y estudios anteriores](https://dortizcormano.com/resume) (CFGS Automatización y Robótica Industrial) participa activamente en la organización de tareas y guiando en la realización de las mismas.

**Adrián Gascón Morán \- Diseñador mecánico**
**1º Ingeniería Mecánica**
Encargado del diseño y modelado 3D de los componentes mecánicos del robot. Con los conocimientos adquiridos en la carrera de mecánica y aprendizaje autodidacta de los programas de diseño 3D.

**Irene Garcia Molina \- Programadora firmware (ESP32 P4)**
**1º Inteligencia Robótica**
Ha recibido formación en programación, diseño y programación de robots, y ha participado en otros proyectos formativos del UJI Robotics. Irene, junto a Victor trabajan en la programación del microcontrolador.

**Raul Garvi Cascos \- Programador software (Raspberry PI)**
**1º, 2º y 3º Inteligencia Robótica**
Gracias a sus conocimientos tras acabar el grado universitario de Ingeniería Informática, Raul se encarga de configurar y programar la Raspberry PI.

**Victor Ioan Gutoi \- Programador firmware (ESP32 P4)**
**1º Inteligencia Robótica**
Experimentado tras trabajar como programador en la empresa GDA (Grupul de Despăgubiri prin Asigurari) trabaja junto a Irene en la programación del microcontrolador.

# Parte 1 \- Diseño y desarrollo del robot {#parte-1---diseño-y-desarrollo-del-robot}

1. ## Planificación y cronograma {#planificación-y-cronograma}

   El desarrollo del robot se ha planificado en fases semanales desde diciembre de 2025 hasta marzo de 2026, organizadas por categorías:

   **Fase 1 — Organización y arranque (Diciembre 2025)**
   - Formación de equipos y evaluación de requisitos (semanas 50–52).
   - Definición de la arquitectura general del proyecto.
   - Inicio de la búsqueda de patrocinadores y elaboración del presupuesto.

   **Fase 2 — Diseño y desarrollo inicial (Enero 2026)**
   - Diseño eléctrico del robot (semanas 1–4).
   - Creación del repositorio GitHub, Trello y proyectos base (semana 2).
   - Desarrollo de componentes esenciales del firmware ESP32 (semanas 2–4): drivers de motores, encoders, cliente MQTT, gestor de telemetría.
   - Desarrollo de contenedores Docker para la Raspberry Pi (semanas 2–3): broker MQTT (Mosquitto), InfluxDB, Telegraf, Grafana.
   - Configuración de la Raspberry Pi y la cámara CSI (semana 2).
   - Desarrollo del sistema de telemetría tanto en firmware como en software (semanas 3–5).

   **Fase 3 — Diseño 3D y visión (Febrero 2026)**
   - Diseño 3D de cada componente individual (semanas 5–8).
   - Diseño 3D de la distribución de componentes en el chasis (semanas 6–9).
   - Diseño 3D de monturas y chasis final (semanas 9–10).
   - Impresión 3D de las piezas (semanas 9–10).
   - Desarrollo del componente de lectura de encoders (semanas 4–6).
   - Desarrollo de visión artificial para detección de curvatura de línea — siguelíneas (semanas 6–10).
   - Desarrollo del componente controlador de servo (semanas 6–10).
   - Inicio del documento de preentrega (semana 8).

   **Fase 4 — Integración y cierre (Marzo 2026)**
   - Conexiones y pruebas eléctricas (semana 10).
   - Desarrollo del componente controlador de motores (semanas 10–11).
   - Desarrollo de componentes pendientes: LiDAR, sensor de línea IR, sensor de consumo y voltaje INA226 (semana 11).
   - Desarrollo de la máquina de estados del firmware (semana 11).
   - Desarrollo del firmware para siguelíneas y sumo (semana 11).
   - Cierre del documento de preentrega parte 1 y parte 2 (semanas 10–11).

2. ## Proceso de diseño del robot {#proceso-de-diseño-del-robot}

   1. ### Diseño mecánico {#diseño-mecánico}

      *\[Fotos y renders incluidos en el documento final\]*

   2. ### Diseño eléctrico {#diseño-eléctrico}

      *\[Esquemáticos y diagramas incluidos en el documento final\]*

3. ## Programación del robot {#programación-del-robot}

   1. ### Arquitectura {#arquitectura}

      El robot utiliza una **arquitectura híbrida de dos procesadores** que separa las funciones de control en tiempo real de las funciones de percepción y estrategia:

      - **ESP32-P4 (Microcontrolador — "El Cerebelo"):** Se encarga de los "reflejos" del robot. Ejecuta el control de bajo nivel de los motores mediante lazos PID, lee los encoders y sensores críticos (IMU, corriente de batería) y reacciona en tiempo real ante eventos como colisiones o pérdida de tracción. Al ser un microcontrolador **determinista**, garantiza que cada lectura de sensor y cada actualización de PWM se ejecute en intervalos exactos sin interferencias del sistema operativo. Si la Raspberry Pi se cuelga o pierde la conexión, el ESP32 puede frenar el robot de forma segura de manera autónoma.

      - **Raspberry Pi 5 (Microprocesador — "El Lóbulo Frontal"):** Se encarga de la estrategia. Procesa la imagen de la cámara CSI (IMX219) mediante OpenCV, ejecuta algoritmos de navegación autónoma con ROS2 y Nav2, y decide "hacia dónde ir". Envía órdenes de alto nivel al ESP32 a través de MQTT (por ejemplo, "avanza a 1 m/s" o "gira 30 grados"). Además, hospeda el sistema completo de monitorización y telemetría (InfluxDB, Grafana).

      - **Comunicación entre ambos:** La conexión se realiza mediante **cable Ethernet directo** entre la RPi5 y el ESP32-P4, utilizando el protocolo **MQTT** sobre TCP/IP. Se eligió Ethernet sobre WiFi para eliminar la latencia variable y garantizar la fiabilidad de las comunicaciones en tiempo real. El broker MQTT (Mosquitto) corre como contenedor Docker en la Raspberry Pi, y tanto el firmware como los servicios de software publican y se suscriben a topics organizados jerárquicamente bajo `robot/`.

      **Distribución de tareas en los núcleos del ESP32-P4:**

      El ESP32-P4 dispone de un sistema de alto rendimiento (HP) con dos núcleos RISC-V a 400 MHz y un sistema de bajo consumo (LP) con un núcleo a 40 MHz. Las tareas FreeRTOS se distribuyen de la siguiente manera:

      | Núcleo | Tarea | Prioridad | Función |
      | :---- | :---- | :---- | :---- |
      | **CPU0 (HP)** | `task_rtcontrol_cpu0` | 10 (máxima) | Control de motores en tiempo real, lectura de encoders, PID, reacciones críticas. No realiza ninguna operación de red. |
      | **CPU1 (HP)** | `task_comms_cpu1` | 10 (máxima) | Gestión MQTT, publicación de telemetría, recepción de comandos, lectura de sensores complejos (IMU, corriente). |
      | **CPU1 (HP)** | `task_monitor_lowpower` | 1 (mínima) | Watchdog de hambruna: publica un heartbeat cada 5 segundos. Si deja de emitir, indica que el sistema está saturado. |

      Esta separación garantiza que el lazo de control de motores en CPU0 **nunca** se vea bloqueado por operaciones de red o parsing de datos, eliminando el jitter en el control PWM.

   2. ### Software {#software}

      El software de la Raspberry Pi 5 se organiza como una pila de **microservicios Docker** orquestados mediante Docker Compose. Cada servicio es independiente, inmutable y configurable exclusivamente a través de variables de entorno definidas en un archivo `.env`. Los servicios se levantan con un único comando: `docker compose up -d --build`.

      **Servicios activos:**

      - **Message Broker (Mosquitto):** Broker MQTT centralizado en el puerto 1883. Gestiona toda la comunicación pub/sub entre el ESP32, los nodos de visión/navegación y el sistema de telemetría. Configurado con acceso anónimo para la red local y persistencia de mensajes activada.

      - **InfluxDB 2.7:** Base de datos de series temporales que almacena todas las métricas del robot. Elegida frente a bases de datos relacionales (MySQL) o archivos planos porque está optimizada para escrituras de alta frecuencia (~100 datos/segundo) y permite consultas temporales eficientes. Almacena los datos en el bucket `local_system` con autenticación por token.

      - **Telegraf:** Agente de recolección de métricas que actúa como puente entre MQTT e InfluxDB. Se suscribe al topic `robot/performance` y parsea los datos en formato JSON extrayendo tags como `task` y `core` para identificar cada núcleo del ESP32. Además, recolecta métricas del sistema de la Raspberry Pi: CPU, memoria, disco, red y temperatura (leyendo `/sys/class/thermal/thermal_zone0/temp`).

      - **Grafana:** Dashboard web (puerto 3000) para visualización en tiempo real de la telemetría. Incluye un dashboard preconfigurado con: gauges de uso de CPU0 y CPU1 del ESP32, temperatura del procesador de la RPi5, gráficas temporales de rendimiento por tarea, vista embebida de la cámara y logs de comandos MQTT. Los dashboards se aprovisionan automáticamente desde archivos JSON versionados en el repositorio.

      **Servicios en desarrollo (contenedores preparados pero desactivados):**

      - **Vision Node:** Nodo ROS2 con OpenCV que captura la cámara CSI (IMX219 8MP) a 820x616 píxeles. Implementa un pipeline de visión artificial para siguelíneas que incluye: umbralización a escala de grises, transformada de perspectiva inversa para obtener una vista cenital (bird's-eye-view) del recorrido, detección de blobs por secciones horizontales (10 secciones) y cálculo de curvatura ponderada. El valor de curvatura se publica en el topic MQTT `robot/curvatura` para que el ESP32 lo utilice como feedforward en el control de dirección. El sistema también ofrece un stream MJPEG por HTTP (puerto 8080) para depuración visual remota.

      - **Navigation Node:** Nodo ROS2 Humble con Nav2 para navegación autónoma. Integra SLAM (localización y mapeo simultáneo) con el LiDAR D200/D500 y planificación de rutas con costmaps locales y globales. Genera comandos de velocidad (`cmd_vel`) que se envían al ESP32 como órdenes de movimiento.

      - **Main Controller:** Backend en Python con FastAPI que actúa como cerebro lógico del sistema. Implementa la máquina de estados del robot a nivel de misión, el puente bidireccional ROS2↔MQTT y una API REST para el dashboard.

   3. ### Firmware {#firmware}

      El firmware del ESP32-P4 está desarrollado con **ESP-IDF** (el framework oficial de Espressif) y sigue una arquitectura modular estricta donde cada funcionalidad se encapsula en un **componente** independiente con su propio `CMakeLists.txt`, cabeceras públicas e implementación privada. Los componentes se compilan como bibliotecas estáticas que se enlazan al binario final.

      **Secuencia de inicialización (`system_init`):**
      1. Inicialización de NVS Flash (almacenamiento no volátil).
      2. Configuración de la interfaz de red y driver Ethernet (RMII/SMI, clock 50 MHz).
      3. Inicialización del códec de audio ES8311 vía I2C y del bus I2S.
      4. Creación de la memoria compartida inter-core con mutex FreeRTOS.
      5. Lanzamiento de `task_comms_cpu1` en CPU1 (retardo de 100ms).
      6. Lanzamiento de `task_rtcontrol_cpu0` en CPU0.

      **Componentes principales del firmware:**

      - **Motors (MCPWM):** Control de dos motores DC JGA25-370 12V mediante el periférico MCPWM del ESP32-P4. Frecuencia de portadora PWM de 20 kHz con resolución de 10 MHz (500 ticks por periodo). Rango de comandos de -1000 a +1000 mapeados a duty cycle. Implementa deadband de 30 ticks (6%) para evitar cortocircuitos de shoot-through en el puente H. Los drivers DRV8871 reciben las señales PWM en los pines GPIO25/26 (motor izquierdo) y GPIO23/5 (motor derecho). El modo de frenado fuerza ambas entradas a HIGH simultáneamente.

      - **Encoder Sensor (PCNT):** Lectura de encoders de cuadratura mediante el periférico hardware PCNT (Pulse Counter), que no consume ciclos de CPU. Configurado en modo X4 (4 cuentas por pulso) con 11 PPR y ratio de reducción 21.3:1 sobre ruedas de 68mm de diámetro. Calcula velocidad lineal (m/s) y distancia acumulada (m) en tiempo real. Resolución aproximada de 0.13 mm por pulso. Los pines de entrada son GPIO18 (canal A) y GPIO19 (canal B). Acumulación en contador software de 64 bits para evitar desbordamiento del contador hardware de 16 bits.

      - **Telemetry Manager:** Gestor de telemetría que empaqueta los datos de sensores en formato **Influx Line Protocol (ILP)** para su ingestión directa en InfluxDB sin transformación. Sigue un patrón builder: `telemetry_create()` → `telemetry_add_float()/add_int()/add_bool()` → `telemetry_destroy()` (que dispara la publicación MQTT). Cada muestra se estampa con un timestamp de microsegundos obtenido de `esp_timer_get_time()`, independientemente del momento en que se publique el bloque. El intervalo de publicación configurado es de 5000 ms, acumulando múltiples muestras de alta frecuencia en un único payload MQTT para evitar la saturación del broker.

      - **MQTT Custom Client:** Capa de abstracción sobre el cliente MQTT de ESP-IDF. Gestiona la conexión con el broker Mosquitto, la publicación en topics (`robot/odometry`, `robot/telemetry`, `robot/events`) y la suscripción a topics de comandos (`robot/cmd`, `robot/curvatura`). Soporta QoS 0 y QoS 1, con un máximo de 4 callbacks de topic registrados simultáneamente. Los callbacks deben ser no bloqueantes (sub-milisegundo) para no romper el Keep-Alive MQTT.

      - **Shared Memory:** Mecanismo de comunicación inter-core protegido por mutex FreeRTOS con timeout de 10 ms. Define dos estructuras compartidas: `robot_sensor_data_t` (velocidad, distancia, corriente, batería, temperatura, contador de encoder) y `robot_command_t` (tipo de comando y parámetros). Incluye contadores de heartbeat por CPU (`heartbeat_cpu0`, `heartbeat_cpu1`) que permiten detectar si un núcleo se ha quedado bloqueado.

      - **State Machine:** Máquina de estados con 7 estados (INIT, WAITING_ORDERS, AUTONOMOUS, REMOTE_CONTROLLED, TELEMETRY_ONLY, MQTT_LOST_ERROR, SHUTDOWN) y 5 modos de operación (NONE, AUTONOMOUS_PATH, AUTONOMOUS_OBSTACLE, REMOTE_DRIVE, TELEMETRY_STREAM). Las transiciones están condicionadas a la conectividad MQTT: si se pierde la conexión durante un modo que la requiere, el robot pasa automáticamente al estado MQTT_LOST_ERROR y detiene los motores. Tras 5000 ms de timeout, intenta reconectar. Los cambios de estado se publican como eventos JSON retenidos en `robot/events`.

      - **Curvature Feedforward:** Módulo que recibe el valor de curvatura calculado por el nodo de visión a través del topic MQTT `robot/curvatura`. Acepta formato texto (ASCII numérico) o binario (4 bytes timestamp + 4 bytes float little-endian). El valor se almacena en una variable global volátil para acceso inmediato desde el lazo de control, permitiendo correcciones anticipadas de dirección antes de que el error sea medido por los encoders.

      - **Audio Player:** Reproductor de sonidos embebidos (PCM 16-bit mono a 16 kHz) a través del códec ES8311 conectado por I2C (GPIO7 SDA, GPIO8 SCL) e I2S (MCLK GPIO13, BCLK GPIO12, WS GPIO10, DOUT GPIO9). Soporta reproducción asíncrona mediante tarea FreeRTOS dedicada con control de volumen (0-100%). Sonidos disponibles: BATTERY_LOW y STARTUP.

      - **Ethernet:** Driver de conectividad Ethernet con soporte para EMAC interno del ESP32-P4 y módulos SPI externos. Gestión de eventos de conexión/desconexión y obtención de IP (DHCP o estática).

      - **Logger:** Sistema centralizado de logging que unifica los mensajes de diagnóstico del firmware.

      - **Performance Monitor:** Módulo que recopila métricas de rendimiento de las tareas FreeRTOS (uso de CPU por tarea y por núcleo) y las publica en formato JSON al topic `robot/performance` para su visualización en Grafana.

4. ## Montaje y construcción {#montaje-y-construcción}

   1. ### Impresión 3D {#impresión-3d}

      *\[Fotos incluidas en el documento final\]*

   2. ### Conexiones eléctricas {#conexiones-eléctricas}

      *\[Fotos incluidas en el documento final\]*

   3. ### Montaje mecánico {#montaje-mecánico}

      *\[Fotos incluidas en el documento final\]*

5. ## Pruebas, validaciones y mejoras {#pruebas,-validaciones-y-mejoras}

   El proceso de pruebas y validación se ha llevado a cabo de forma incremental, verificando cada subsistema de forma aislada antes de la integración completa.

   **Pruebas realizadas:**

   - **Conectividad Ethernet y MQTT:** Se verificó la comunicación bidireccional entre el ESP32-P4 y la Raspberry Pi 5 a través de cable Ethernet directo. Se validó la conexión al broker Mosquitto, la publicación y suscripción a topics, y la reconexión automática tras desconexiones simuladas. Se comprobó que los eventos de conexión/desconexión se registran correctamente en `robot/events`.

   - **Pipeline de telemetría completo:** Se validó la cadena ESP32 → MQTT (Influx Line Protocol) → Mosquitto → Telegraf → InfluxDB → Grafana. Se verificó que los datos de rendimiento por tarea y por núcleo del ESP32 llegan correctamente al dashboard de Grafana con las etiquetas `task` y `core`. Se comprobó que las métricas del sistema de la RPi5 (CPU, temperatura, memoria, disco) se recolectan cada 5 segundos.

   - **Control de motores (MCPWM):** Se verificó la generación correcta de señales PWM a 20 kHz en los pines de los drivers DRV8871. Se probaron los modos de avance, retroceso y frenado con el patrón de test implementado (-1000 durante 1.5s en bucle). Se comprobó que el deadband de 30 ticks previene transitorios peligrosos al cambiar de dirección.

   - **Lectura de encoders (PCNT):** Se validó la lectura de encoders de cuadratura en modo X4 mediante el periférico hardware PCNT. Se verificó que el cálculo de velocidad (m/s) y distancia acumulada (m) es coherente con las especificaciones mecánicas (11 PPR, reducción 21.3:1, rueda de 68mm). Se comprobó la correcta acumulación en el contador software de 64 bits.

   - **Máquina de estados:** Se verificaron las transiciones entre estados (INIT → WAITING_ORDERS → modos operativos) y la caída automática a MQTT_LOST_ERROR al perder la conexión MQTT. Se comprobó la publicación de eventos JSON retenidos en cada cambio de estado.

   - **Visión artificial (feedforward de curvatura):** Se probó el pipeline de visión con la cámara IMX219: captura a 820x616, transformada de perspectiva, detección de blobs por secciones y cálculo de curvatura ponderada. Se verificó la publicación del valor de curvatura en `robot/curvatura` y su recepción por el módulo feedforward del firmware.

   - **Sistema de audio:** Se validó la reproducción de sonidos embebidos (STARTUP, BATTERY_LOW) a través del códec ES8311 con control de volumen.

   **Pruebas pendientes y plan de validación:**

   - **Integración PID de velocidad con encoders:** Cerrar el lazo de control sustituyendo el patrón de test hardcodeado por un controlador PID que use la velocidad medida por los encoders como retroalimentación. Ajustar las constantes Kp, Ki, Kd experimentalmente.

   - **Integración de visión con control de motores:** Conectar la curvatura calculada por el nodo de visión con el lazo de control del firmware para lograr un siguelíneas funcional en pista real. Validar la latencia del pipeline completo (cámara → procesamiento → MQTT → actuación).

   - **Integración del LiDAR D500:** Implementar el driver de comunicación UART con el LiDAR y validar el mapa de puntos 360° para navegación autónoma con Nav2.

   - **Sensor de corriente y voltaje INA226:** Implementar la lectura I2C del INA226 para monitorización de batería en tiempo real. Calibrar los umbrales de apagado seguro.

   - **Sensor de línea IR (array de 8 canales):** Integrar el array IR como entrada analógica de error para el PID de seguimiento de línea. Calibrar sobre superficie real.

   - **Pruebas de autonomía completa:** Validar el robot en un circuito de prueba con todos los subsistemas integrados. Medir la autonomía de la batería LiPo 4S 2300mAh bajo carga real.

   - **Pruebas de robustez y recuperación:** Simular fallos de comunicación, pérdida de alimentación parcial y situaciones de emergencia para validar que la máquina de estados reacciona correctamente en todos los casos.

6. ## Presupuesto {#presupuesto}

   El presupuesto total del proyecto se ha optimizado buscando los proveedores más económicos para cada componente, priorizando AliExpress para componentes electrónicos y proveedores europeos para elementos críticos o urgentes.

   **Resumen por categoría:**

   | Categoría | Coste estimado |
   | :---- | :---- |
   | Ordenadores (Raspberry Pi 5 8GB) | 81,70 € |
   | Controladores (ESP32-P4-ETH) | 17,96 € |
   | Sensores (LiDAR D500, IMX219, INA226, IR array) | 67,30 € |
   | Actuadores (2x JGA25-370, MG996R, altavoz) | 23,94 € |
   | Módulos electrónica (drivers DRV8871, buck XL4015) | 6,14 € |
   | Baterías (LiPo 4S 2300mAh) | 30,99 € |
   | Cableado y conectores | 6,17 € |
   | Materiales y mecánica (PLA, ruedas) | 8,80 € |
   | **TOTAL** | **243,00 €** |

   Este presupuesto considera los precios más económicos por proveedor estándar. Con proveedores afiliados (BricoGeek, Filament2Print, Mouser), el total asciende a 276,69 €. El gasto efectivo fuera de afiliados es de tan solo 6,05 € (cable ethernet y ruedas giratorias), ya que la mayoría de componentes se cubren con los patrocinios o se adquieren a través de proveedores colaboradores.

   Varios componentes ya se encontraban en posesión del equipo antes del inicio del proyecto (motores JGA25-370, altavoces, portafusibles, interruptores, conector 2-a-6, convertidores buck, cámara IMX219, array IR, ESP32-P4-ETH y PLA), lo que reduce el gasto real necesario a aproximadamente 189,50 €.

7. ## Financiación {#financiación}

   El proyecto cuenta con el apoyo de tres patrocinadores:

   - **BRICOGEEK:** Tienda online de electrónica y robótica que proporciona componentes y módulos electrónicos para el desarrollo del robot.
   - **JLCPCB:** Fabricante de PCBs que colabora con servicios de fabricación de placas de circuito impreso para posibles diseños de PCB personalizados.
   - **UJILAB Innovació:** Laboratorio de innovación de la Universitat Jaume I que proporciona acceso a herramientas, espacios de trabajo y recursos de fabricación (impresoras 3D, herramientas de taller).

   Adicionalmente, parte del material ha sido adquirido directamente por los miembros del equipo con fondos propios, y se buscan activamente acuerdos con proveedores como Mouser Electronics y RobotShop para obtener descuentos educativos.

# Parte 2 \- Desafío "Automatización el futuro" {#parte-2---desafío-"automatización-el-futuro"}

1. ## Definición del proceso a mejorar {#definición-del-proceso-a-mejorar}

   **Transporte de material entre líneas de producción en entornos de fabricación industrial.**

   En las fábricas modernas de manufactura electrónica e industrial — como las plantas de Schneider Electric — el flujo de materiales entre distintas zonas de producción es una operación constante y crítica. Por ejemplo, en una planta donde una zona se dedica a la fabricación de PCBs y otra zona al ensamblaje del producto final, existe un flujo continuo de bandejas, cajas y contenedores de componentes que deben moverse de una línea a otra.

   Actualmente, este transporte inter-líneas se realiza de varias formas:
   - **Manualmente**, con operarios que empujan carros o llevan bandejas, lo que consume tiempo productivo y es propenso a errores de entrega.
   - **Con carretillas elevadoras (toros/transpaletas)**, que están sobredimensionadas para cargas ligeras y representan un riesgo de seguridad en zonas con personal.
   - **Con AGVs de gran tamaño tipo transpaleta**, que son soluciones costosas (decenas de miles de euros) diseñadas para mover palés completos, cuando en muchos casos la carga real entre líneas son cajas o bandejas de pocos kilogramos.

   El problema concreto que queremos abordar es el **transporte de cargas ligeras y medianas (1–15 kg) entre estaciones de trabajo dentro de una misma planta de producción**. Este proceso es repetitivo, predecible (las rutas entre líneas son fijas) y no requiere la capacidad de carga de una transpaleta industrial. Sin embargo, consume horas de trabajo humano al día y genera cuellos de botella cuando las líneas se quedan sin material.

   Nuestra propuesta se centra en el transporte **intra-fábrica entre líneas de producción**, un escenario presente en toda planta de fabricación industrial.

2. ## Descripción de la solución robótica propuesta {#descripción-de-la-solución-robótica-propuesta}

   Proponemos una **flota coordinada de robots móviles ligeros** capaces de transportar material entre líneas de producción de forma autónoma, siguiendo rutas predefinidas marcadas en el suelo de la fábrica.

   **Concepto de operación:**

   Cada robot de la flota opera de forma autónoma siguiendo rutas marcadas con líneas en el suelo (similares a las "carreteras" internas que ya existen en muchas fábricas para AGVs y transpaletas). Los robots recogen bandejas o cajas de una estación de origen y las transportan a una estación de destino, siguiendo un circuito predefinido. Un sistema central de gestión asigna las misiones de transporte y coordina la flota para evitar colisiones y optimizar los tiempos de entrega.

   **Capacidades del robot aplicadas a este escenario:**

   - **Seguimiento de línea (siguelíneas):** El array IR de 8 canales y la cámara IMX219 con el algoritmo de detección de curvatura permiten al robot seguir las rutas marcadas en el suelo con precisión, incluyendo curvas y bifurcaciones. El sistema de feedforward de curvatura anticipa los giros antes de llegar a ellos, permitiendo una conducción suave sin frenazos.

   - **Navegación autónoma con LiDAR:** El LiDAR D500 proporciona un mapa de 360° del entorno, permitiendo al robot detectar obstáculos inesperados (personas, otros robots, carretillas) y detenerse o esquivarlos de forma segura. Combinado con Nav2 y SLAM, el robot puede navegar entre estaciones incluso en tramos sin línea marcada.

   - **Telemetría y monitorización en tiempo real:** El dashboard Grafana permite a los operarios de planta ver en todo momento la posición, estado de batería, carga transportada y estado de cada robot de la flota. Las alertas automáticas avisan si un robot se queda parado, pierde la conexión o tiene batería baja.

   - **Comunicación MQTT para coordinación de flota:** El protocolo MQTT permite escalar de un solo robot a una flota coordinada de forma natural. Un gestor central publica misiones de transporte y cada robot informa de su estado, posición y disponibilidad. La latencia baja del Ethernet garantiza una coordinación en tiempo real.

   - **Máquina de estados con recuperación de fallos:** Si un robot pierde la conexión MQTT o detecta un fallo, automáticamente se detiene en posición segura y notifica al sistema. Esto previene accidentes y permite que otro robot de la flota asuma la misión pendiente.

   **Ventajas frente a soluciones existentes:**

   - **Coste reducido:** Cada unidad de la flota cuesta ~250 €, frente a los miles de euros de un AGV industrial tipo transpaleta. Esto permite desplegar múltiples unidades por el precio de un solo robot grande.
   - **Agilidad:** Robots pequeños y ligeros que pueden circular por pasillos estrechos entre líneas de producción sin interferir con el personal.
   - **Escalabilidad:** Añadir un nuevo robot a la flota solo requiere conectarlo al broker MQTT; el sistema de gestión lo detecta automáticamente.
   - **Flexibilidad:** Las rutas se pueden reconfigurar cambiando las líneas en el suelo, sin necesidad de reprogramar software complejo ni instalar infraestructura cara (raíles, imanes, reflectores).

3. ## Modificaciones o adaptaciones al diseño original {#modificaciones-o-adaptaciones-al-diseño-original}

   Para adaptar el robot del ASTI Challenge al escenario de transporte inter-líneas en fábrica, se contemplan las siguientes modificaciones:

   - **Plataforma de carga:** Añadir una bandeja o soporte superior al chasis para transportar cajas y bandejas de componentes. El servo MG996R puede utilizarse como mecanismo de retención o liberación de la carga en las estaciones de recogida y entrega.

   - **Refuerzo de motores:** Para cargas superiores a 5 kg, sustituir los motores JGA25-370 por motores de mayor par manteniendo la misma interfaz de driver (DRV8871). Alternativamente, aumentar la relación de reducción de la reductora.

   - **Gestión de flota (software):** Desarrollar un servicio adicional en Docker (en la Raspberry Pi o en un servidor central) que implemente la lógica de asignación de misiones: cola de pedidos de transporte, asignación al robot más cercano disponible y gestión de prioridades.

   - **Identificación de estaciones:** Utilizar la cámara IMX219 para leer códigos QR o ArUco markers en cada estación de trabajo, permitiendo al robot confirmar que ha llegado al destino correcto antes de depositar la carga.

   - **Ampliación de autonomía:** Para turnos de trabajo largos (8h), añadir una batería LiPo de mayor capacidad o implementar un sistema de carga por contacto en las estaciones de espera, aprovechando la monitorización del INA226 para gestionar la rotación de robots según nivel de batería.
