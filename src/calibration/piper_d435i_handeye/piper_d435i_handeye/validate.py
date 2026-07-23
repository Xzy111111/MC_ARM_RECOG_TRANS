#!/usr/bin/env python3
#
# 拖拽验证：在不同位姿下采集标定板的位姿，
# 计算在 base_link 坐标系下的标准差，
# 判断标定精度。
#
# 使用前需要：
#   1. ros2 launch piper_d435i_handeye publish.launch.py
#   2. 相机 + aruco_single 在运行
#

import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException


class CalibrationValidator(Node):
    def __init__(self):
        super().__init__('calibration_validator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples = []  # 存储 (x, y, z, qx, qy, qz, qw)
        self.sample_count = 0

        self._timer = self.create_timer(0.5, self.check_marker)

    def check_marker(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'camera_marker', rclpy.time.Time())
        except LookupException:
            return
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return

        p = t.transform.translation
        q = t.transform.rotation

        print(
            f'\r  [current]  '
            f'x={p.x:+.4f}  y={p.y:+.4f}  z={p.z:+.4f}  '
            f'  samples={self.sample_count}',
            end='', flush=True)

    def take_sample(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'camera_marker', rclpy.time.Time())
        except LookupException:
            print('\n[ERROR] camera_marker TF not available. Is aruco_single running?')
            return False
        except Exception as e:
            print(f'\n[ERROR] {e}')
            return False

        p = t.transform.translation
        q = t.transform.rotation
        self.samples.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.sample_count += 1
        print(f'\n  → Sample #{self.sample_count} captured')
        return True

    def compute_and_report(self):
        if len(self.samples) < 3:
            print('\n[ERROR] Need at least 3 samples to validate.')
            return

        arr = np.array(self.samples)
        pos = arr[:, :3]
        mean_pos = pos.mean(axis=0)
        std_pos = pos.std(axis=0)
        max_dev = np.max(np.linalg.norm(pos - mean_pos, axis=1))

        print('\n' + '=' * 60)
        print('  Calibration Validation Report')
        print('=' * 60)
        print(f'  Total samples: {len(self.samples)}')
        print(f'\n  Marker position in base_link (should be constant):')
        print(f'    Mean:  x={mean_pos[0]:.4f}  y={mean_pos[1]:.4f}  z={mean_pos[2]:.4f}')
        print(f'    Std:   x={std_pos[0]:.4f}  y={std_pos[1]:.4f}  z={std_pos[2]:.4f}')
        print(f'    Max deviation from mean: {max_dev:.4f} m')

        # 判断标准
        pos_error = np.linalg.norm(std_pos)
        if pos_error < 0.005:
            print(f'\n  [EXCELLENT] Position std < 5 mm → calibration is very good')
        elif pos_error < 0.015:
            print(f'\n  [GOOD] Position std < 15 mm → calibration is acceptable')
        elif pos_error < 0.030:
            print(f'\n  [FAIR] Position std < 30 mm → could be better')
        else:
            print(f'\n  [POOR] Position std > 30 mm → recalibrate recommended')
        print('=' * 60)

    def run_interactive(self):
        print('=' * 60)
        print('  Calibration Validation - Drag Robot Arm Method')
        print('=' * 60)
        print('  1) Move the robot arm to a new pose')
        print('  2) Press ENTER to capture a sample')
        print('  3) Repeat at least 5-10 different poses')
        print('  4) Type "done" and press ENTER to see results')
        print('  5) Type "quit" to exit')
        print('=' * 60)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                line = input().strip().lower()
            except EOFError:
                break

            if line == 'quit':
                break
            elif line == 'done':
                self.compute_and_report()
                break
            elif line == '':
                self.take_sample()
            else:
                print(f'  Unknown command: {line}')


def main(args=None):
    rclpy.init(args=args)
    validator = CalibrationValidator()
    try:
        validator.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
