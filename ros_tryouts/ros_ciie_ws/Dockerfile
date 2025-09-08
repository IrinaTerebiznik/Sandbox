# Imagen base de ROS 2 Humble para ARM64 (Raspberry Pi 4/5 con Ubuntu 22.04)
FROM ros:humble-ros-base-jammy

# Instalar dependencias de Python y GPIO
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Instalar utilidades de compilación de ROS 2
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Instalar librerías de Python que quieras usar en nodos
RUN pip install --no-cache-dir \
    rclpy \
    gpiozero

# Copiar tu paquete (motor_gpio) dentro del workspace
# (asume que tenés motor_gpio en ./src)
COPY ./src ./src

# Compilar el workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Configuración de entorno por defecto
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Entry point para lanzar ROS 2
CMD ["bash"]
