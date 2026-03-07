# Folder Structure (Estructura del Proyecto de Software Docker/SBC)

> **Nota de Nomenclatura:** La arquitectura de directorios de este proyecto requiere que **todos los nombres de carpetas y archivos estén en inglés**. El contenido explicativo en prosa de los archivos puede estar en español o inglés libremente.

Todo el software subyacente que opera sobre la placa Single-Board Computer (Raspberry Pi 5) se encapsula mediante contenedores Docker para ser Inmutable, Portátil y Versionable. El Host Linux se usa meramente como anfitrión de red (`network_mode: "host"`).

La raíz se aloja bajo `02-software/` y orquestada por el archivo `docker-compose.yml`.

## Microservicios Lógicos

### `01-vision/` y `02-navigation/`
Contienen configuraciones, Launch-files o códigos fuente que resuelven los retos de visión artificial y de SLAM con ROS 2.
- Estos directorios montan dinámicamente `/src` hacia dentro de los contenedores Docker mediante Bind-Mounts (`volumes`). Esto permite Hot-Reloading de _Behavior Trees_ o _Python Scripts_ en tiempo real sin requerir una re-compilación de la imagen de Sistema Operativo entera para depurar.
- Operan bajo un aislamiento mitigado en el Host (usando `ROS_DOMAIN_ID=13` para prevenir contaminación Multicast de ROS a otras redes estudiantiles/oficinas).

### `05-telemetry/`
Guarda de forma persistente a través del tiempo las configuraciones estrictas, reglas estáticas, y paneles de las tres herramientas de Observabilidad:
- **`telegraf/telegraf.conf`**: El despachador; contiene suscripciones al broker MQTT y los formateadores/parsers diseñados para digerir directamente nuestro Influx Line Protocol (ILP) hacia adentro de la DB.
- **`grafana/provisioning/`**: Define la fuente de datos Influx "Por Código". Impide que tengas que iniciar sesión manualmente y configurar conexiones HTTP visualmente.
- **`grafana/dashboards/`**: Modelos JSON en crudo exportados desde la Interfaz Gráfica de Grafana. Su existencia aquí permite replicar tu robot desde 0 y mantener tus gráficas completas si la tarjeta SD se corrompe.

### `06-mqtt-broker/`
El cimiento neutral para comunicaciones intra-robot inter-proceso.
- **`mosquitto/mosquitto.conf`**: Bloquea las directivas de seguridad (Access Control Lists Listners) del contenedor general de Eclipse. Configura de forma rígida el modo anónimo local del Puerto 1883 para ingestas de hiper-alta capacidad emitidas desde el ESP32, asumiendo una Intranet cerrada (Cable Ethernet Directo ESP32 <-> RPi5).

### Globales
- **`docker-compose.yml`**: Mapa maestro topológico. Inicializa el Grandmaster Clock `linuxptp`, los brokers, las bases de datos `influxdb` y las consolas de Grafana inyectando los secretos mediante `.env`.
- **`.env`**: Archivo de entorno nativo del Host donde descansan los Token/Passwords Maestros de InfluxDB y Administradores Grafana (`INFLUXDB_TOKEN`, `GRAFANA_ADMIN_PASSWORD`), asegurando que dichas claves nunca queden commit-eadas al código fuente global de ROS2/Firmware.
