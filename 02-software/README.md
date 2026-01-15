Todos los servicios se levantan desde el `docker-compose.yml` situado en la
raíz del repositorio.

---

## 01 – Vision

Contiene los nodos ROS2 encargados de la captura y procesamiento de imagen
(OpenCV y/o IA).

**Responsabilidades:**

- Captura de cámara (IMX219)
- Procesado de imagen (RAW, máscaras, IA)
- Publicación de resultados en ROS2

**Tecnologías:**

- ROS2 Humble
- OpenCV
- libcamera (Raspberry Pi)

---

## 02 – Navigation

Gestiona la navegación autónoma del robot.

**Responsabilidades:**

- SLAM
- Planificación de rutas
- Integración con sensores (LiDAR, odometría)

**Tecnologías:**

- ROS2 Humble
- Nav2
- LiDAR D200

---

## 03 – Backend

Actúa como cerebro lógico del sistema.

**Responsabilidades:**

- Máquina de estados del robot
- Puente ROS2 ↔ MQTT
- Comunicación con el ESP32-P4
- Streaming de datos hacia el dashboard

**Tecnologías:**

- Python
- FastAPI
- MQTT
- ROS2

---

## 04 – Dashboard

Interfaz gráfica para el usuario.

**Responsabilidades:**

- Visualización del estado del robot
- Telemetría
- Eventos y alertas

**Tecnologías:**

- React
- TypeScript
- Servido mediante contenedor Docker

---

## 05 – Infra

Servicios de infraestructura necesarios para el sistema.

**Incluye:**

- Broker MQTT
- Recolección de métricas
- Persistencia de datos

Estos servicios no contienen lógica del robot, pero son esenciales para su
funcionamiento y monitorización.

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
