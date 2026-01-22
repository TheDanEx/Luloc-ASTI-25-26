# 07-mqtt-broker/Dockerfile
FROM eclipse-mosquitto:2.0

# Copia config si existe
COPY src/mosquitto.conf /mosquitto/config/mosquitto.conf:ro

EXPOSE 1883
