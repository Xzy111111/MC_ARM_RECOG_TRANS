import atexit
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from vehicle_driver.config import CMD_VEL_TOPIC, MAX_VX, MAX_VY, MAX_WZ


PUBLISH_RATE_HZ = 10.0
DEFAULT_LINEAR_SPEED = 0.60
DEFAULT_ANGULAR_SPEED = 1.00


class KeyboardTestNode(Node):
    def __init__(self):
        super().__init__('keyboard_test')
        self._state_lock = threading.Lock()
        self._running = True

        self._max_linear = DEFAULT_LINEAR_SPEED
        self._max_angular = DEFAULT_ANGULAR_SPEED

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        self._pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

        if sys.stdin.isatty():
            atexit.register(self._restore_terminal)
            self._print_help()
            t = threading.Thread(target=self._kb_loop, daemon=True)
            t.start()
        else:
            self.get_logger().warning('stdin not a TTY, keyboard disabled')

    def _print_help(self):
        print()
        print('  u i o    前左 / 前进 / 前右')
        print('  j k l    左移 / 停止 / 右移')
        print('  m , .    后左 / 后退 / 后右')
        print('  a d      左转 / 右转')
        print('  Space/k  停止')
        print('  w/x      线速度 +/- 10%')
        print('  e/c      角速度 +/- 10%')
        print('  q        退出')
        print()

    def _kb_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while rclpy.ok() and self._running:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                key = sys.stdin.read(1).lower()
                if not key:
                    continue
                self._handle_key(key)
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write('\r\n')
            sys.stdout.flush()

    def _print_status(self):
        sys.stdout.write(
            f'\r  vx={self._vx:+.2f} vy={self._vy:+.2f} wz={self._wz:+.2f}  '
            f'lin={self._max_linear:.2f} ang={self._max_angular:.2f}  '
        )
        sys.stdout.flush()

    def _handle_key(self, key: str):
        with self._state_lock:
            if key == 'i':
                self._vx, self._vy, self._wz = self._max_linear, 0.0, 0.0
            elif key == ',':
                self._vx, self._vy, self._wz = -self._max_linear, 0.0, 0.0
            elif key == 'j':
                self._vx, self._vy, self._wz = 0.0, self._max_linear, 0.0
            elif key == 'l':
                self._vx, self._vy, self._wz = 0.0, -self._max_linear, 0.0
            elif key == 'u':
                self._vx, self._vy, self._wz = self._max_linear, self._max_linear, 0.0
            elif key == 'o':
                self._vx, self._vy, self._wz = self._max_linear, -self._max_linear, 0.0
            elif key == 'm':
                self._vx, self._vy, self._wz = -self._max_linear, self._max_linear, 0.0
            elif key == '.':
                self._vx, self._vy, self._wz = -self._max_linear, -self._max_linear, 0.0
            elif key == 'a':
                self._vx, self._vy, self._wz = 0.0, 0.0, self._max_angular
            elif key == 'd':
                self._vx, self._vy, self._wz = 0.0, 0.0, -self._max_angular
            elif key in ('k', ' '):
                self._vx = self._vy = self._wz = 0.0
            elif key == 'w':
                self._max_linear = min(MAX_VX, self._max_linear * 1.1)
            elif key == 'x':
                self._max_linear = max(0.01, self._max_linear * 0.9)
            elif key == 'e':
                self._max_angular = min(MAX_WZ, self._max_angular * 1.1)
            elif key == 'c':
                self._max_angular = max(0.05, self._max_angular * 0.9)
            elif key == 'q':
                self._vx = self._vy = self._wz = 0.0
                self._running = False

        if key == 'q':
            self._publish()
            self._print_status()
            if rclpy.ok():
                rclpy.shutdown()
            return

        self._print_status()
        if key in ('i', ',', 'j', 'l', 'u', 'o', 'm', '.', 'a', 'd', 'k', ' '):
            self._publish()

    def _publish(self):
        with self._state_lock:
            vx, vy, wz = self._vx, self._vy, self._wz
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._pub.publish(msg)

    def _restore_terminal(self):
        pass

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTestNode()
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
