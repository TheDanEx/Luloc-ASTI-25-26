# ASTI Challenge 2025/26

# UJI Robotics Luloc Team

Proyecto universitario de robótica móvil basado en:

- **ESP32-P4** para control en tiempo real
- **Raspberry Pi 5** para percepción, navegación e interfaz
- **Dashboard y Telemetria** monitorización a tiempo real

## Arquitectura general

- El ESP32-P4 ejecuta el control de bajo nivel (motores, sensores, batería).
- La RPi5 ejecuta percepción (visión), navegación (Nav2) y la interfaz de usuario.
- La comunicación entre ambos se realiza mediante **MQTT**.
- Cada bloque funcional en la RPi5 corre en su propio contenedor Docker.

## Estructura

- `01-firmware/` → Código ESP32-P4 (ESP-IDF, multinuecleo)
- `02-software/` → Servicios Docker en la RPi5
- `03-assets/` → Archivos externos y media

## Ejecución

```bash
docker compose up -d --build
```
