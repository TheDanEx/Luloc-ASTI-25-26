# Navigation Node (navigation_node)

## Propósito Arquitectónico
Servicio central dedicado al control robótico general: Localización, Mapeo, evasión de obstáculos de trayectorias (SLAM/Nav2) y control general de estados en la máquina que sirve como "Cerebro de Flota" interactuando con algoritmos path/planning.

## Entorno y Dependencias
Dependencia base probable a paquete oficial ROS2 Nav2 u framework de custom planner. Invocado mediante Dockerfile dentro de `./02-navigation`. Trabaja en el mismo `ROS_DOMAIN_ID=13`. 

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** N/A Directamente. (Extrae datos de red en modo Host).
- **Software:** Consumidor masivo de Tópicos (Sensores, Odometría de ESP32) y Publicador estricto (`cmd_vel` o en su lugar MQTT). Totalmente libre del aislamiento de redes docker al estar en modo `host`.

## Flujo de Ejecución Lógico
Al arrancar, monta su ecosistema base de ROS 2. Construye Costmaps (Mapas de costes globales y locales) fusionando LIDAR, odometría en rueda y frames de cámara en paralelo. Lazo de control lento para path globales (1Hz) y muy rápido para correctores DWA locales (10-20Hz).

## Parámetros de Configuración y Entorno (Docker)
- `ROS_DOMAIN_ID`: (`13`) Permite descubrir y hacer P2P DDS instantáneo e ininterrumpido con el Vision Node y otros publicadores ROS.
- `network_mode: "host"`: Requerido de forma casi mandataria en stacks ROS Nav para evitar pérdidas de latencia en transformaciones y broadcast de tópicos TF de alta frecuencia.
- `volumes`: Vinculación local del área de trabajo `./02-navigation/src` a `/root/navigation_node`, idéntico a visión para poder sobreescribir behavior trees sin reconstruir la imagen de ROS entera.

## Puntos Críticos y Depuración
- **Desincronizaciones de TF (Transforms):** Si hay retrasos del ESP-P4 de más de medio segundo entregando odometría, o los timestamp están desencajados, la nube del Costmap quedará inválida provocando paros de la base o rotaciones sobre sí mismo del robot.
- **Carga de CPU Sostenida:** Fallos en la creación del local costmap que saturen la Raspberry provocando demoras a otras apps, rompiendo la arquitectura time-safe general de MQTT.
