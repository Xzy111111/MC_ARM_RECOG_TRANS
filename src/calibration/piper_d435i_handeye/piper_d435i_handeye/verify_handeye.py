#!/usr/bin/env python3

# ============================================================
# verify_handeye.py
# 验证手眼标定结果：加载已保存的标定文件，打印变换矩阵，
# 并以静态 TF 形式发布标定结果。
# ============================================================

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from easy_handeye2.handeye_calibration import load_calibration


class HandeyeVerifier(Node):
    """加载并验证手眼标定结果。"""

    def __init__(self):
        super().__init__('handeye_verifier')

        self.declare_parameter('name', 'piper_d435i_eih')
        name = self.get_parameter('name').get_parameter_value().string_value

        self.get_logger().info(f'Loading calibration: {name}')

        try:
            self.calibration = load_calibration(name)
        except FileNotFoundError:
            self.get_logger().error(
                f'Calibration file for "{name}" not found.\n'
                f'  Expected at: ~/.ros2/easy_handeye2/calibrations/{name}.calib\n'
                f'  Run handeye_calibrate.launch.py first to perform calibration.'
            )
            sys.exit(1)

        parameters = self.calibration.parameters
        transform = self.calibration.transform

        # ── 打印标定参数 ──
        info = (
            f'\n'
            f'  Calibration name:       {parameters.name}\n'
            f'  Calibration type:       {parameters.calibration_type}\n'
            f'  Robot base frame:       {parameters.robot_base_frame}\n'
            f'  Robot effector frame:   {parameters.robot_effector_frame}\n'
            f'  Tracking base frame:    {parameters.tracking_base_frame}\n'
            f'  Tracking marker frame:  {parameters.tracking_marker_frame}\n'
            f'  Freehand movement:      {parameters.freehand_robot_movement}\n'
        )
        self.get_logger().info(info)

        # ── 打印变换矩阵 ──
        t = transform
        self.get_logger().info(
            f'Calibrated transform ({parameters.robot_effector_frame} -> '
            f'{parameters.tracking_base_frame}):\n'
            f'  Translation:  x={t.translation.x:.6f}  y={t.translation.y:.6f}  '
            f'z={t.translation.z:.6f}\n'
            f'  Rotation (quaternion):  '
            f'x={t.rotation.x:.6f}  y={t.rotation.y:.6f}  '
            f'z={t.rotation.z:.6f}  w={t.rotation.w:.6f}\n'
        )

        # ── 发布静态变换到 TF ──
        if parameters.calibration_type == 'eye_in_hand':
            orig = parameters.robot_effector_frame
        else:
            orig = parameters.robot_base_frame
        dest = parameters.tracking_base_frame

        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_ts = geometry_msgs.msg.TransformStamped()
        static_ts.header.stamp = self.get_clock().now().to_msg()
        static_ts.header.frame_id = orig
        static_ts.child_frame_id = dest
        static_ts.transform = transform

        self._broadcaster.sendTransform(static_ts)

        self.get_logger().info(
            f'Published static transform: {orig} -> {dest}\n'
            f'Verification complete.'
        )


def main(args=None):
    rclpy.init(args=args)
    verifier = HandeyeVerifier()
    try:
        rclpy.spin_once(verifier, timeout_sec=1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
