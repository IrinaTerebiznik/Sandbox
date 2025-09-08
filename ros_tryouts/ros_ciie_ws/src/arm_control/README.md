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

## 2) Montaje en Docker

Asegurate de:
- Pasar el dispositivo serie al contenedor.
- Exportar las variables de entorno.
- Dar permisos (grupo `dialout` o equivalente según distro).

### Ejemplo `docker run` (ajustá el dispositivo real):
```bash
docker run --rm -it \
  --network host \
  --device=/dev/ttyACM0 \
  -e USB_PORT=/dev/ttyACM0 \
  -e BAUDRATE=115200 \
  -v /tu/ws:/ws \
  tu-imagen-ros2-humble
