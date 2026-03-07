# InfluxDB (influx-db)

## Propósito Arquitectónico
Motor Time-Series Database (Base de datos de series temporales) responsable del almacenamiento crudo e indexado de métricas logísticas y telemetría de ultra-alta frecuencia (temperaturas, consumos, codificadores) arrojada por la flota o vehículo.

## Entorno y Dependencias
Imagen pública de `influxdb:2.7`. Totalmente interconectada con la configuración mediante variables del fichero `.env` inyectado a los secretos de Docker (`INFLUXDB_USER`, `INFLUXDB_BUCKET`, etc). Depende a nivel almacenamiento de un Virtual Volume de Docker puro (`influxdb-data`).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Persistencia de estado vinculada a almacenamiento masivo (SSD de la Single Board Computer) mapeado en `/var/lib/influxdb2`.
- **Software:** Expone el end-point TCP `8086:8086` de su API HTTP. Se testea ciclicamente mediante un `curl` local interno (Healthcheck). Ingiere datos masivamente desde Telegraf sin intervención directa.

## Flujo de Ejecución Lógico
Al lanzar la imagen, inicia el clúster interno Influx 2.0 y verifica si existen credenciales previas; de no ser así, usa las variables `setup` generadas inicialmente por las claves preconfiguradas y levanta la API de carga masiva de paquetes.

## Parámetros de Configuración y Entorno (Docker)
La inicialización desatendida recibe las credenciales "Master" del sistema extraídas del OS a través del fichero `.env`:
- `DOCKER_INFLUXDB_INIT_MODE`: Fijado en `setup`. Activa el bootstrap interno para poblar la DB base.
- `DOCKER_INFLUXDB_INIT_USERNAME` / `_PASSWORD`: Extraídos de `${INFLUXDB_USER}` y `${INFLUXDB_PASSWORD}` respectivamente.
- `DOCKER_INFLUXDB_INIT_ORG`: Espacio organizativo, pre-establecido por `${INFLUXDB_ORG}`.
- `DOCKER_INFLUXDB_INIT_BUCKET`: La "base de datos" o cubeta por defecto (`${INFLUXDB_BUCKET}`).
- `DOCKER_INFLUXDB_INIT_ADMIN_TOKEN`: API Token universal (`${INFLUXDB_TOKEN}`) vital porque será consumido asimétricamente por Telegraf para ingesta y Grafana para el display.
- `ports`: `"8086:8086"` inyectado y expuesto nativamente en la SBC.
- `volumes`: Disco Virtual Persistente de Docker (`influxdb-data`). Los datos aquí logran no borrarse ni perdiendo la imagen y no acoplados en capeta relativa local.

## Puntos Críticos y Depuración
- **Desgaste Prematuro Flash (SD Cards):** Si el intervalo de recolección de Telegraf es ridículamente corto (1ms), la Base de datos sobrescribirá las pistas de la tarjeta SD o eMMC reduciendo su vida a escasos meses y rompiendo el Journal del OS Host (SBC).
- **Caída Estática / Crash Loops:** Si el disco se llena al 100% de métricas no borradas automáticamente por una directiva "Retention Policy", la instancia jamás superará el Healthcheck paralizando la instrumentación entera ("CrashLoopBackOff").
