# Imagen base de ROS 2 Humble para ARM64 (Raspberry Pi 4/5 con Ubuntu 22.04)
FROM ros:humble-ros-base-jammy

# Instalar dependencias de Python
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Instalar utilidades de compilación de ROS 2
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copiar tu paquete (motor_gpio) dentro del workspace
COPY ./src ./src

# Compilar el workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build

# Configuración de entorno por defecto (para cada sesión bash)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Entry point para lanzar ROS 2
CMD ["bash"]
