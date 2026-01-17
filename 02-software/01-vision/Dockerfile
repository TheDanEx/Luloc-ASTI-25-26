# 01-vision/Dockerfile
# Base ROS2 Humble (Debian Jammy)
FROM ros:humble-ros-base-jammy

# Evitar preguntas interactivas
ENV DEBIAN_FRONTEND=noninteractive

# Actualiza e instala dependencias
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    v4l-utils \
    ros-humble-usb-cam \
    ros-humble-image-tools \
    && rm -rf /var/lib/apt/lists/*

# Instala cv_bridge y Flask
RUN pip3 install cv_bridge flask

# Carpeta de trabajo dentro del contenedor
WORKDIR /root/vision_node

# Copia el script de streaming al contenedor
COPY src/vision.py .

# Configura ROS2 en todos los shells
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Comando por defecto al iniciar el contenedor
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 /root/vision_node/vision.py"]
