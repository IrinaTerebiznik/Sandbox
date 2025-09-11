import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

try:
    import serial
except ImportError:
    serial = None  # Permitimos correr sin pyserial instalado (modo sin-serial)

CMD_MAP = {
    'saludar': 'S',
    'girar': 'G',
    'reverencia': 'R',
    'stop': 'X',
}

def normalize_cmd(raw: str) -> str:
    s = raw.strip().lower()
    # aceptar también códigos de una letra desde serial/web
    inv = {v.lower(): k for k, v in CMD_MAP.items()}
    if s in CMD_MAP:
        return s
    if s in inv:
        return inv[s]
    # también aceptar "cmd:xxx"
    if s.startswith('cmd:'):
        cand = s.split(':', 1)[1].strip()
        if cand in CMD_MAP:
            return cand
        if cand.lower() in inv:
            return inv[cand.lower()]
    return ''

class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')

        # -------- Config (ENV + parámetros) --------
        default_port = ''
        default_baud = 115200

        env_port = os.getenv('USB_PORT', default_port).strip()
        env_baud = os.getenv('BAUDRATE', str(default_baud)).strip()

        self.declare_parameter('port', env_port if env_port else default_port)
        self.declare_parameter('baud', int(env_baud) if env_baud.isdigit() else default_baud)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Serial habilitado solo si hay puerto definido y pyserial disponible
        self.serial_enabled = bool(self.port) and (serial is not None)

        # -------- Publishers / Subscribers --------
        self.sub_cmd = self.create_subscription(String, '/arm/command', self.on_cmd, 10)
        self.pub_status = self.create_publisher(String, '/arm/status', 10)
        # eco de comandos para UI/web
        self.pub_cmd_echo = self.create_publisher(String, '/arm/command_echo', 10)

        self.serial = None
        self._run_reader = False
        self.reader_thread = None

        if self.serial_enabled:
            self._open_serial()
            self._run_reader = True
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            self.get_logger().info(f'Serial habilitado → {self.port} @ {self.baud}')
        else:
            reason = []
            if not self.port:
                reason.append("USB_PORT no especificado")
            if serial is None:
                reason.append("pyserial no instalado")
            msg = " / ".join(reason) if reason else "serial deshabilitado"
            self.get_logger().warn(f'Serial DESHABILITADO ({msg}). El nodo funcionará solo por tópicos.')

        self.get_logger().info('ArmSerialBridge listo. Subs: /arm/command | Pubs: /arm/status, /arm/command_echo')

    # ---------- Serial handling ----------
    def _open_serial(self):
        while self.serial is None and self.serial_enabled:
            try:
                self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(0.3)  # estabilizar
                self._status(f'Conectado a {self.port} @ {self.baud}')
            except Exception as e:
                self._status(f'No se pudo abrir {self.port}: {e}. Reintentando en 1s…', warn=True)
                time.sleep(1)

    def _reader_loop(self):
        while self._run_reader:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if line:
                        # Todo lo que venga del Arduino lo publicamos como status
                        self.pub_status.publish(String(data=line))
                        # Si el Arduino envía algo que parezca comando, lo reflejamos a echo
                        cmd = normalize_cmd(line)
                        if cmd:
                            self.pub_cmd_echo.publish(String(data=cmd))
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f'Lectura serial falló: {e}. Reintentando…')
                self.serial = None
                self._open_serial()

    # ---------- ROS callbacks ----------
    def on_cmd(self, msg: String):
        raw = msg.data
        cmd = normalize_cmd(raw)
        if not cmd:
            self._status(f'Comando inválido: "{raw}". Válidos: {list(CMD_MAP.keys())}', warn=True)
            return

        # publicar eco para la UI siempre (serial esté o no)
        self.pub_cmd_echo.publish(String(data=cmd))
        self._status(f'CMD recibido: {cmd}')

        if not self.serial_enabled:
            # modo dry-run: solo log/echo
            self._status('Serial deshabilitado: ejecutando en modo dry-run (no se envía a Arduino).', warn=True)
            return

        # enviar a Arduino
        code = CMD_MAP[cmd] + '\n'
        try:
            if not self.serial or not self.serial.is_open:
                self._status('Serial no abierta, reabriendo…', warn=True)
                self.serial = None
                self._open_serial()
            self.serial.write(code.encode('ascii'))
            self._status(f'Enviado a serial: {cmd} → {CMD_MAP[cmd]}')
        except Exception as e:
            self._status(f'Error enviando por serial: {e}', error=True)
            self.serial = None  # fuerza reconexión próximo intento

    # ---------- Helpers ----------
    def _status(self, text: str, warn: bool = False, error: bool = False):
        if error:
            self.get_logger().error(text)
        elif warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        # siempre publicamos también al tópico de status para la UI
        self.pub_status.publish(String(data=text))

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
