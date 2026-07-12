import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    import serial
except ImportError:
    serial = None

from vehicle_driver.config import (
    SERIAL_PORT, BAUDRATE, SERIAL_TIMEOUT_SEC,
    SEND_RATE_HZ, CMD_TIMEOUT_SEC, SERIAL_RECONNECT_PERIOD_SEC,
    MAX_VX, MAX_VY, MAX_WZ,
    CMD_VEL_TOPIC, DEBUG_PRINT_FRAME,
    VX_SCALE, VY_SCALE, WZ_SCALE,
)

FRAME_HEAD = 0xAA
FRAME_TAIL = 0x55
FRAME_LEN = 24


def _velocity_to_count(value: float, scale: float) -> int:
    return max(-660, min(660, round(value * scale)))


def build_command_frame(vx: float, vy: float, vw: float, enable: bool = True, scales: tuple = (660.0, 660.0, 660.0)) -> bytes:
    frame = bytearray(FRAME_LEN)
    frame[0] = FRAME_HEAD
    frame[23] = FRAME_TAIL
    vx_i = _velocity_to_count(vx, scales[0])
    vy_i = _velocity_to_count(vy, scales[1])
    vw_i = _velocity_to_count(vw, scales[2])
    frame[1:3] = vx_i.to_bytes(2, 'big', signed=True)
    frame[3:5] = vy_i.to_bytes(2, 'big', signed=True)
    frame[5:7] = vw_i.to_bytes(2, 'big', signed=True)
    if enable:
        frame[7] = 0x01
        frame[8] = 0x01
    else:
        frame[7] = 0x02
        frame[8] = 0x00
    return bytes(frame)


def _format_frame_hex(frame: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in frame)


class VehicleDriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver')
        self._lock = threading.Lock()

        self._port = self.declare_parameter('port', SERIAL_PORT).value
        self._baudrate = self.declare_parameter('baudrate', BAUDRATE).value
        self._send_rate_hz = self.declare_parameter('send_rate_hz', SEND_RATE_HZ).value
        self._cmd_timeout_sec = self.declare_parameter('cmd_timeout_sec', CMD_TIMEOUT_SEC).value
        self._max_vx = self.declare_parameter('max_vx', MAX_VX).value
        self._max_vy = self.declare_parameter('max_vy', MAX_VY).value
        self._max_wz = self.declare_parameter('max_wz', MAX_WZ).value
        self._debug_print = self.declare_parameter('debug_print_frame', DEBUG_PRINT_FRAME).value
        self._vx_scale = self.declare_parameter('vx_scale', VX_SCALE).value
        self._vy_scale = self.declare_parameter('vy_scale', VY_SCALE).value
        self._wz_scale = self.declare_parameter('wz_scale', WZ_SCALE).value

        self._serial = self._open_serial()

        self._vx = 0.0
        self._vy = 0.0
        self._vw = 0.0
        self._last_cmd_time = self.get_clock().now()

        self._sub = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._cmd_vel_callback, 10)

        period = 1.0 / max(1.0, self._send_rate_hz)
        self._send_timer = self.create_timer(period, self._on_send_timer)
        self._reconnect_timer = self.create_timer(
            SERIAL_RECONNECT_PERIOD_SEC, self._on_reconnect_timer)

        self.get_logger().info(f'vehicle_driver started, sub: {CMD_VEL_TOPIC}')

    def _open_serial(self):
        if serial is None:
            self.get_logger().error('python3-serial not installed')
            return None
        try:
            ser = serial.Serial(self._port, self._baudrate,
                                timeout=SERIAL_TIMEOUT_SEC)
            self.get_logger().info(f'Serial opened: {self._port} @ {self._baudrate}')
            return ser
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            return None

    def _cmd_vel_callback(self, msg: Twist):
        with self._lock:
            self._vx = max(-self._max_vx, min(self._max_vx, msg.linear.x))
            self._vy = max(-self._max_vy, min(self._max_vy, msg.linear.y))
            self._vw = max(-self._max_wz, min(self._max_wz, msg.angular.z))
            self._last_cmd_time = self.get_clock().now()

    def _on_send_timer(self):
        with self._lock:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age > self._cmd_timeout_sec:
                vx = vy = vw = 0.0
            else:
                vx, vy, vw = self._vx, self._vy, self._vw

        frame = build_command_frame(vx, vy, vw, scales=(self._vx_scale, self._vy_scale, self._wz_scale))
        if self._debug_print:
            self.get_logger().info(f'TX: {_format_frame_hex(frame)}')

        ser = self._serial
        if ser is None or not ser.is_open:
            return
        try:
            ser.write(frame)
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _on_reconnect_timer(self):
        ser = self._serial
        if ser is not None and ser.is_open:
            return
        self._serial = self._open_serial()

    def destroy_node(self):
        if hasattr(self, '_serial') and self._serial and self._serial.is_open:
            try:
                self._serial.write(build_command_frame(0, 0, 0, scales=(self._vx_scale, self._vy_scale, self._wz_scale)))
            except Exception:
                pass
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
