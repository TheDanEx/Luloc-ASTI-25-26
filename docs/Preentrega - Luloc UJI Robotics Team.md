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

      El robot utiliza una **arquitectura híbrida de dos procesadores** que separa el control en tiempo real de la percepción y la estrategia:

      - **ESP32-P4 (microcontrolador):** Control determinista de motores, lectura de encoders y sensores críticos (IMU, corriente). Si la Raspberry Pi falla, el ESP32 puede frenar el robot de forma autónoma.

      - **Raspberry Pi 5 (microprocesador):** Procesamiento de cámara con OpenCV, navegación autónoma con ROS2/Nav2, y sistema completo de telemetría y monitorización (InfluxDB, Grafana). Envía órdenes de alto nivel al ESP32.

      **Decisión clave — MQTT en lugar de micro-ROS:**

      Inicialmente queríamos usar **micro-ROS** para integrar el ESP32 directamente con el ecosistema ROS2 de la Raspberry Pi. Sin embargo, **no existe todavía una implementación estable de micro-ROS para el ESP32-P4**, que es un chip muy reciente. Intentar portarlo habría consumido semanas sin garantía de éxito.

      La solución fue usar **MQTT**, un protocolo de mensajería más ligero y probado. Aunque perdemos la integración directa con ROS2, ganamos una comunicación fiable que pudimos poner en marcha en pocos días. La conexión se hace por **cable Ethernet directo** (no WiFi) para eliminar latencia variable.

      **Distribución de tareas en el ESP32-P4:**

      El ESP32-P4 dispone de dos núcleos RISC-V a 400 MHz. Los repartimos así:

      | Núcleo | Función |
      | :---- | :---- |
      | **CPU0** | Control de motores en tiempo real y lectura de encoders. No realiza operaciones de red para evitar bloqueos. |
      | **CPU1** | Comunicaciones MQTT, publicación de telemetría, recepción de comandos y lectura de sensores secundarios (IMU, corriente). |

      Además, una tarea de vigilancia con prioridad mínima envía un heartbeat cada 5 segundos. Si deja de emitir, indica que el sistema está sobrecargado.

   2. ### Software {#software}

      El software de la Raspberry Pi 5 se organiza como **microservicios Docker** orquestados con Docker Compose, lo que permite arrancar todo con un solo comando y aislar cada servicio.

      **Reto: telemetría en tiempo real**

      Un reto inicial fue cómo almacenar y visualizar los cientos de datos por segundo que genera el robot. Una base de datos convencional no era viable por la frecuencia de escritura. La solución fue montar una cadena especializada en series temporales:

      1. **Mosquitto (MQTT Broker)** — Recibe los datos del ESP32 y los distribuye a los servicios que los necesiten.
      2. **Telegraf** — Recoge mensajes MQTT y los ingesta en la base de datos. También monitoriza la salud de la propia Raspberry Pi (temperatura, CPU, memoria).
      3. **InfluxDB 2.7** — Base de datos de series temporales, optimizada para escrituras de alta frecuencia y consultas como "¿qué velocidad tenía la rueda hace 3 minutos?".
      4. **Grafana** — Dashboard web con gráficas en tiempo real: uso de CPU por núcleo del ESP32, temperatura, vista embebida de la cámara y logs de comandos.

      Esta cadena nos ha permitido diagnosticar problemas que de otro modo serían invisibles, como picos de uso de CPU al publicar datos MQTT o subidas de temperatura de la RPi5 al procesar vídeo.

      **Servicios en desarrollo (preparados pero desactivados):**

      - **Visión:** Nodo Python con OpenCV que captura la cámara CSI (IMX219), aplica transformada de perspectiva, detecta la línea del suelo y calcula la curvatura del recorrido. Publica el valor al ESP32 vía MQTT para feedforward de dirección. Un problema que encontramos fue que las librerías de la cámara CSI (libcamera/picamera2) no son triviales de configurar dentro de Docker, lo cual nos obligó a probar tanto dentro como fuera del contenedor.

      - **Navegación:** Nodo ROS2 Humble con Nav2 que integra el LiDAR D500 para SLAM y planificación de rutas. Contenedor preparado pero pendiente de integración.

      - **Controlador principal:** Backend en Python (FastAPI) que implementa la máquina de estados a nivel de misión, el puente ROS2↔MQTT y una API REST para el dashboard.

   3. ### Firmware {#firmware}

      El firmware del ESP32-P4 está desarrollado con **ESP-IDF** y sigue una arquitectura modular: cada funcionalidad es un componente independiente con su propia carpeta. Al principio del proyecto perdimos tiempo intentando desarrollar todo en un solo archivo y rápidamente nos dimos cuenta de que era inmanejable. La modularidad nos permite trabajar varias personas en paralelo y probar cada pieza por separado.

      **Secuencia de arranque:**

      El firmware sigue un orden estricto de inicialización: NVS Flash → Ethernet → audio (ES8311) → memoria compartida → tareas FreeRTOS. Este orden importa: si el cliente MQTT arrancaba antes de que Ethernet estuviera listo, el sistema se colgaba. Descubrir este problema nos costó varias sesiones de depuración con el monitor serie.

      **Componentes principales y problemas encontrados:**

      - **Control de motores (MCPWM):** Los motores DC JGA25-370 se controlan mediante PWM a 20 kHz a través de drivers DRV8871. Al probar cambios bruscos de dirección, se producían picos de corriente que reiniciaban el ESP32. Lo solucionamos implementando un deadband de 30 ticks (6% del periodo) que inserta una pausa entre cambios de sentido, evitando shoot-through en el puente H.

      - **Encoders (PCNT):** Lectura de encoders de cuadratura mediante el periférico hardware PCNT (sin consumo de CPU). Con 11 PPR, reductora 21.3:1 y ruedas de 68mm, conseguimos ~0.13mm de resolución por pulso. Inicialmente el robot marcaba el doble de distancia real: el error estaba en la configuración del modo de conteo (X4 vs X2). También implementamos un acumulador de 64 bits porque el contador hardware de 16 bits se desbordaba en recorridos largos.

      - **Telemetría (Influx Line Protocol):** Enviar cada dato individual por MQTT saturaría la red. La solución fue empaquetar por lotes: el firmware acumula muestras con timestamp de microsegundos y las envía en un solo payload cada 5 segundos. La red recibe pocos mensajes, pero cada dato conserva su instante exacto de medición.

      - **Cliente MQTT:** Gestiona publicación (odometría, telemetría, eventos) y suscripción (comandos, curvatura). Un problema recurrente fue que callbacks lentos rompían el keep-alive MQTT. Aprendimos a mantenerlos por debajo del milisegundo, delegando el trabajo pesado a las tareas principales.

      - **Memoria compartida inter-core:** Los dos núcleos intercambian datos (sensores y comandos) a través de estructuras protegidas por mutex con timeout de 10 ms. Incluye contadores de heartbeat por CPU para detectar bloqueos del otro núcleo.

      - **Máquina de estados:** 7 estados (INIT, WAITING_ORDERS, AUTONOMOUS, REMOTE_CONTROLLED, TELEMETRY_ONLY, MQTT_LOST_ERROR, SHUTDOWN). Si se pierde la conexión MQTT durante un modo que la requiere, el robot detiene los motores y entra en error seguro. Tras 5 segundos, intenta reconectar. Cada transición se publica como evento JSON retenido.

      - **Feedforward de curvatura:** Recibe del nodo de visión el valor de curvatura del camino y lo pone a disposición del lazo de control para anticipar los giros antes de que los encoders detecten la desviación.

      - **Audio:** Reproducción de sonidos embebidos (arranque, batería baja) vía códec ES8311. Tuvimos fallos intermitentes en la inicialización I2C por resistencias pull-up inadecuadas.

      - **Monitor de rendimiento:** Mide el uso de CPU por tarea FreeRTOS y lo publica en JSON a Grafana. Fue clave para detectar que la tarea de comunicaciones consumía más recursos de lo esperado al formatear JSON.

4. ## Montaje y construcción {#montaje-y-construcción}

   1. ### Impresión 3D {#impresión-3d}

      *\[Fotos incluidas en el documento final\]*

   2. ### Conexiones eléctricas {#conexiones-eléctricas}

      *\[Fotos incluidas en el documento final\]*

   3. ### Montaje mecánico {#montaje-mecánico}

      *\[Fotos incluidas en el documento final\]*

5. ## Pruebas, validaciones y mejoras {#pruebas,-validaciones-y-mejoras}

   Hemos seguido un enfoque incremental: probar cada subsistema de forma aislada antes de la integración. A continuación describimos las pruebas realizadas y los problemas encontrados.

   **Pruebas realizadas:**

   - **Conectividad Ethernet y MQTT:** La conexión se caía intermitentemente al principio. El problema era que MQTT intentaba conectarse antes de que Ethernet tuviera IP. Se solucionó añadiendo una espera a la obtención de dirección. Se verificó también la reconexión automática tras desconexiones simuladas.

   - **Pipeline de telemetría:** Se validó la cadena completa ESP32 → Mosquitto → Telegraf → InfluxDB → Grafana. Un problema fue que los datos llegaban a Grafana con timestamp incorrecto: se estaba usando la hora de llegada al servidor en vez de la marca temporal del ESP32. Se corrigió la configuración de Telegraf para respetar los timestamps de origen.

   - **Control de motores:** Se verificaron las señales PWM a 20 kHz con osciloscopio. Al cambiar de dirección bruscamente, picos de corriente reiniciaban el ESP32. Se implementó el deadband (6%) que previene shoot-through. Actualmente funcionan con un patrón de prueba fijo; pendiente conectar al PID.

   - **Encoders:** Los valores de distancia iniciales no cuadraban (marcaba el doble). El error estaba en la configuración del modo de conteo X4 vs X2. Tras corregir la fórmula, los cálculos coincidieron con mediciones manuales.

   - **Máquina de estados:** Se simularon pérdidas de conexión desconectando el cable Ethernet. El robot detiene motores automáticamente, entra en error seguro y reconecta tras 5 segundos. Cada transición queda registrada como evento MQTT.

   - **Visión artificial:** Se validó el pipeline completo: captura, transformada de perspectiva, detección de línea y cálculo de curvatura. La calibración de perspectiva variaba según posición de la cámara, así que desarrollamos una herramienta interactiva de recalibración. El valor de curvatura llega correctamente al ESP32 vía MQTT.

   - **Audio:** Funcionó tras solucionar fallos intermitentes en la inicialización I2C del códec ES8311 (resistencias pull-up inadecuadas).

   **Problemas transversales del proceso:**

   - **Gestión de compras:** Los tiempos de envío de AliExpress (3-4 semanas) nos obligaron a hacer pedidos antes de tener el diseño cerrado, asumiendo riesgos en la elección de componentes. Componentes de proyectos anteriores aliviaron el presupuesto.

   - **ESP32-P4 como chip nuevo:** Poca documentación y ejemplos en la comunidad. Funcionalidades bien documentadas en otros ESP32 requerían investigación adicional para el P4. Esto ralentizó el desarrollo inicial pero nos obligó a entender el hardware a mayor profundidad.

   - **micro-ROS descartado:** No existe implementación estable para el ESP32-P4, por lo que se optó por MQTT como alternativa viable (ver sección de arquitectura).

   **Pruebas pendientes y plan de validación:**

   - **Cerrar el lazo de control PID:** Actualmente los motores funcionan con un patrón de test fijo. El siguiente paso es implementar el controlador PID que use la velocidad medida por los encoders como retroalimentación, y ajustar las constantes experimentalmente sobre el robot real.

   - **Integrar visión con motores:** Conectar el valor de curvatura de la cámara con el control de dirección del firmware para lograr un siguelíneas funcional en pista real. Medir la latencia total del pipeline (cámara → procesamiento → MQTT → actuación en motores).

   - **Integrar el LiDAR:** Implementar la comunicación con el LiDAR D500 y validar el mapa de 360° para la navegación autónoma.

   - **Sensor de batería (INA226):** Programar la lectura del sensor de corriente y voltaje para monitorizar la batería en tiempo real y configurar un apagado seguro cuando el nivel sea crítico.

   - **Sensor de línea IR:** Integrar el array de 8 sensores infrarrojos como entrada alternativa al PID de seguimiento de línea, calibrándolo sobre la superficie real de la pista.

   - **Prueba de autonomía completa:** Validar el robot con todos los subsistemas integrados en un circuito de prueba y medir cuánto dura la batería LiPo 4S 2300mAh bajo carga real.

6. ## Presupuesto {#presupuesto}

   Uno de los mayores retos del proyecto ha sido ajustar el diseño a un presupuesto limitado. Para cada componente, comparamos precios en múltiples proveedores (AliExpress, Mouser Electronics, RobotShop, Amazon, Farnell) y elaboramos una hoja de cálculo con todas las opciones.

   **Proceso de selección de componentes:**

   Las decisiones de compra no fueron solo económicas, también técnicas. Por ejemplo, para el microcontrolador evaluamos el STM32F411 (~4 €), el STM32F407 (~18 €) y el ESP32-P4-ETH (~18 €). Elegimos el ESP32-P4 porque incluye Ethernet integrado (necesario para comunicarse con la Raspberry Pi sin WiFi), dos núcleos de alto rendimiento y un amplio ecosistema de desarrollo (ESP-IDF). Para los motores, descartamos los servos Dynamixel (40–300 € por unidad) porque necesitábamos rotación continua a alta velocidad, no posicionamiento preciso, y los motores DC JGA25-370 (~10 €) cumplían perfectamente a una fracción del coste.

   Un problema recurrente fue la **gestión de tiempos de envío**: AliExpress ofrece los mejores precios pero con plazos de 3-4 semanas, lo que nos obligó a hacer pedidos antes de tener el diseño completamente cerrado. En algunos casos compramos componentes que luego no usamos (como un driver TB9051FTG que sustituimos por el DRV8871 por disponibilidad). Para componentes urgentes o críticos, recurrimos a proveedores europeos (Mouser, RobotShop) asumiendo un coste mayor.

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

   Varios componentes ya se encontraban en posesión del equipo de proyectos anteriores (motores, altavoces, portafusibles, interruptores, convertidores buck, cámara, array IR, ESP32-P4 y PLA), lo que reduce el gasto real necesario a aproximadamente 189,50 €. Con proveedores afiliados y patrocinadores, el gasto efectivo de bolsillo se reduce a tan solo 6,05 €.

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
