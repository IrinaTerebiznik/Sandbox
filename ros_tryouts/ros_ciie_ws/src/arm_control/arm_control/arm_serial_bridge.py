import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, serial.tools.list_ports
import threading, time

CMD_MAP = {
    'saludar': 'S',
    'girar': 'G',
    'reverencia': 'R',
    'stop': 'X',
}

class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')
        # Defaults
        default_port = '/dev/ttyUSB0'
        default_baud = 115200

        # Declaro parámetros ROS (por si querés overridearlos vía --ros-args)
        self.declare_parameter('port', os.getenv('USB_PORT', default_port))
        self.declare_parameter('baud', int(os.getenv('BAUDRATE', str(default_baud))))

        # Leo valor final de parámetros
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.serial = None
        self._open_serial()

        self.sub = self.create_subscription(String, '/arm/command', self.on_cmd, 10)
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        self._run_reader = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            f'ArmSerialBridge listo. Escuchando /arm/command → {self.port} @ {self.baud}'
        )

    def _open_serial(self):
        while self.serial is None:
            try:
                self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(0.3)
                self.get_logger().info(f'Conectado a {self.port} @ {self.baud}')
            except Exception as e:
                self.get_logger().warn(
                    f'No se pudo abrir {self.port}: {e}. Reintentando en 1s…'
                )
                time.sleep(1)

    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in CMD_MAP:
            self.get_logger().warn(f'Comando inválido: "{cmd}". Válidos: {list(CMD_MAP.keys())}')
            return
        code = CMD_MAP[cmd] + '\n'
        try:
            if not self.serial or not self.serial.is_open:
                self.get_logger().warn('Serial no abierta, reabriendo…')
                self.serial = None
                self._open_serial()
            self.serial.write(code.encode('ascii'))
            self.get_logger().info(f'Enviado: {cmd} → {CMD_MAP[cmd]}')
        except Exception as e:
            self.get_logger().error(f'Error enviando por serial: {e}')
            self.serial = None

    def _reader_loop(self):
        while self._run_reader:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if line:
                        self.status_pub.publish(String(data=line))
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f'Lectura serial falló: {e}. Reintentando…')
                self.serial = None
                self._open_serial()

    def destroy_node(self):
        self._run_reader = False
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    node = ArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
