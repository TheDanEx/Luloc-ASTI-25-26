# Firmware ESP32-P4

Firmware desarrollado con **ESP-IDF** usando VSCode + IDF-IDE.

El ESP32-P4 se utiliza como controlador de tiempo real del robot.

## Uso de núcleos

- **Core 0 (HP – Tiempo real)**  
  Control de motores, PID, lectura de encoders y reacciones críticas.

- **Core 1 (HP – Lógica y comunicaciones)**  
  Sensores complejos (IMU, corriente), parsing MQTT, preparación de datos.

- **LP Core (Low Power)**  
  Monitorización de batería, watchdogs, tareas de bajo consumo.

## Comunicación

- MQTT sobre Ethernet con la Raspberry Pi 5
- Publica telemetría y recibe órdenes

El firmware está diseñado para ser **determinista**, evitando bloqueos y lógica pesada.
