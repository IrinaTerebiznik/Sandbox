# arm_control

Bridge ROS 2 ↔ Arduino por **serial** para disparar **comandos predefinidos** de un brazo robótico: `saludar`, `girar`, `reverencia`, y `stop`.

- **ROS 2**: Humble
- **Topic de entrada**: `/arm/command` (`std_msgs/String`)
- **Topic de salida (logs/feedback)**: `/arm/status` (`std_msgs/String`)
- **Serial**: puerto y baudrate configurables por **variables de entorno** o **parámetros ROS**.

## 1) Variables de entorno

El nodo lee:

- `USB_PORT` → ej. `/dev/ttyACM0` o `/dev/ttyUSB0`
- `BAUDRATE` → ej. `115200`

Si no están definidas, usa los parámetros ROS `port` y `baud` (que también tienen default).

## 2) Luego de montar el  Docker

 ```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws && colcon build && source install/setup.bash
# correr el agente
ros2 run arm_control arm_serial_bridge
 ```

 ## 3) Testeo de conectividad

 Desde otra terminal, habre una tmux console, luego:
  ```bash
ros2 topic pub -1 /arm/command std_msgs/String "{data: saludar}"
ros2 topic echo /arm/command_echo
ros2 topic echo /arm/status
 ```