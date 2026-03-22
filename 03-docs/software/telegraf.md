# Telegraf

## Propósito Arquitectónico
Agente puente ultra-ligero actuando como ETL en tiempo real (Extracción, Transformación y Carga). Toma en crudo todos los JSON o payloads numéricos provenientes del MQTT-Broker y los ingesta masivamente en masa optimizada hacia InfluxDB de forma asíncrona.

## Entorno y Dependencias
Depende estrictamente de múltiples orígenes: Ficheros de configuración locales montados `05-telemetry/telegraf:/etc/telegraf` (ro), scripts dinámicos, lectura agresiva de `/sys/class/thermal` del Host para reportar calor propio. Y lo más vital: sólo inicia si el Healthcheck de InfluxDB devuelve `service_healthy`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Volumen mapeado hacia `/sys/class/thermal` del Host.
- **Host (Local Inputs):** Captura de métricas nativas del sistema (CPU, Memoria, Disco, y tráfico de red segmentado en Ethernet `eth*`/`en*` y Wi-Fi `wlan*`/`wl*`).
- **Software (MQTT Inputs):**
  - `robot/telemetry/#`: Formato **Influx Line Protocol (ILP)** con precisión de nanosegundos (19 dígitos).
  - `robot/events`: Formato **JSON** para eventos críticos.
  - `robot/logs/#`: Formato **JSON** para logs remotos estructurados.
  - `robot/performance`: Formato JSON (Legacy/Task stats).
- **Software (Outputs):** Ingestor agresivo hacia InfluxDB `8086` mediante el API v2 Batch.

## Flujo de Ejecución Lógico
Dependiendo del plugin habilitado en su `telegraf.conf` escondido en la carpeta montada, el demonio arranca y captura todos los tópicos relevantes de `robot/#`. Intercepta los strings en crudo con su Input MQTT, los agiliza en memoria y cada ciclo de milisegundos (`TELEGRAF_COLLECTION_INTERVAL`) formatea todo al protocolo de línea de Influx e inserta ráfagas TCP sobre el puerto 8086 limitando la apertura de transacciones por evento evitando ahogamientos.

## Parámetros de Configuración y Entorno (Docker)
Este manejador carece de puertos porque es un servicio unilateral (Poll / Push) pasivo. Se amolda al `.env` para adquirir en tiempo de diseño:
- `INFLUXDB_HOST`: URI hacia el puerto nativo (o nombre contenedor) como `${INFLUXDB_HOST}` (Típicamente `http://influxdb:8086` o idéntico).
- `INFLUXDB_TOKEN`: Token `${INFLUXDB_TOKEN}` extraído de las variables de entorno para saltar las barreras de Auth hacia la base de datos Influx generada antes.
- `INFLUXDB_ORG` y `INFLUXDB_BUCKET`: Destinos lógicos explícitos dentro de esa instancia de InfluxDB.
- `TELEGRAF_COLLECTION_INTERVAL`: Resolución general del ciclo global de ingestión telemétrica `${TELEGRAF_COLLECTION_INTERVAL}` (fijando tiempos de sleep interno pasivos entre polls al Mosquitto o el Host).
- `volumes`: Carpeta estática mapeada `./05-telemetry/telegraf` donde se hospeda el archivo general de orquestado (`telegraf.conf`), archivos bash utilitarios de `/scripts:ro`, y telemetría de disco `thermal` en vivo extraída engañando al contenedor (`/sys/class/thermal`).

## Puntos Críticos y Depuración
- **Parser de Errores Silenciosos:** Si un componente del ESP32 envía telemetría corrupta de un float como un `NaN` o un "String", Telegraf fallará esa línea individual, y si no hay verbose login habilitado se perderán cientos de métricas en Influx sin trazo explicativo perdiendo visibilidad a lo largo de un test. Reemplazar y parsear fuertemente del lado Edge en la CPU.
