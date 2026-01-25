# 04-backend/Dockerfile
FROM python:3.11-slim

WORKDIR /app

# Dependencias básicas
RUN pip install --no-cache-dir fastapi uvicorn paho-mqtt

COPY src/ .

EXPOSE 51113

CMD ["bash", "-c", "echo 'Contenedor listo para desarrollo backend' && bash"]
