# 02-navigation/Dockerfile
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# Dependencias ROS2 para navegación
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/navigation_node

COPY src/ .

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup bringup_launch.py"]
