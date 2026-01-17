# 04-backend/Dockerfile
FROM python:3.11-slim

WORKDIR /app

# Instala dependencias básicas
RUN pip install --no-cache-dir fastapi uvicorn paho-mqtt

COPY src/ .

EXPOSE 51113

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "51113"]
