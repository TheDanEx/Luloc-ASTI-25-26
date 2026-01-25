# Luloc ASTI 25/26 – Software Stack

Este repositorio contiene el software del sistema robótico, organizado en
módulos independientes y desplegado mediante **Docker Compose**.

Todos los servicios se levantan desde el `docker-compose.yml` situado en la
raíz del repositorio.

---

## 01 – Vision

Procesamiento de imagen y visión artificial.

**Responsabilidades:**

- Captura de cámara CSI (IMX219 – Raspberry Pi 5)
- Procesado de imagen con OpenCV
- Extracción de datos (features, estados, detecciones)
- Publicación de resultados (ROS2 / MQTT)

**Tecnologías:**

- Python
- OpenCV
- GStreamer
- libcamera
- (ROS2 solo para datos, no imagen)

---

## 02 – Navigation

Navegación autónoma del robot.

**Responsabilidades:**

- SLAM
- Planificación de rutas
- Integración con sensores (LiDAR, odometría)

**Tecnologías:**

- ROS2 Humble
- Nav2
- LiDAR D200

---

## 03 – Dashboard

Interfaz gráfica para el usuario.

**Responsabilidades:**

- Visualización del estado del robot
- Telemetría en tiempo real
- Eventos y alertas

**Tecnologías:**

- React
- TypeScript
- Docker

---

## 04 – Backend

Cerebro lógico del sistema.

**Responsabilidades:**

- Máquina de estados del robot
- Puente ROS2 ↔ MQTT
- Comunicación con ESP32
- API para el dashboard

**Tecnologías:**

- Python
- FastAPI
- ROS2
- MQTT

---

## 05 – Time Series Database

Base de datos de series temporales.

**Responsabilidades:**

- Persistencia de métricas
- Telemetría histórica

**Tecnologías:**

- InfluxDB 2.x

---

## 06 – Metrics Agent

Recolección de métricas del sistema y servicios.

**Responsabilidades:**

- Métricas de sistema
- Envío a InfluxDB

**Tecnologías:**

- Telegraf

---

## 07 – Message Broker

Broker de mensajería del sistema.

**Responsabilidades:**

- Comunicación entre backend, visión, ESP32 y servicios auxiliares

**Tecnologías:**

- MQTT
- Mosquitto

---

## Ejecución

Desde la raíz del repositorio:

```bash
docker compose up -d --build
```

Para detener el sistema:

```bash
docker compose down
```
