#!/usr/bin/env python3
#
# AR_bringup.launch.py
# 一键启动 Piper 机械臂手眼标定所需的所有节点。
#
# 启动内容：
#   1. Realsense D435i 相机驱动
#   2. Piper 机械臂 CAN 控制器（需 can0 正常且臂上电）
#   3. robot_state_publisher + joint_state_publisher_gui（滑块控制）
#   4. 静态 TF: link6 → camera_link（手眼标定初始估计）
#   5. rviz2（显示 Camera 图像 + TF 标记帧，无 RobotModel）
#   6. ArUco 二维码检测
#   7. EasyHandEye 手眼标定 GUI
#
# 使用方式：
#   ros2 launch bringup AR_bringup.launch.py
#
# 机械臂启动前准备：
#   1. CAN 接口： sudo ip link set can0 up type can bitrate 1000000
#   2. 确认机械臂已上电且 CAN 线已连接
#   3. 确认 /dev/video* 设备节点存在（Realsense 相机已连接）
#

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ============================================================
    # 1. Realsense D435i 相机驱动
    # ============================================================
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ])
        ]),
    )

    # ============================================================
    # 2. Piper 机械臂 CAN 控制器
    #
    # 参考参数（来自 start_single_agx_arm.launch.py）：
    #   arm_type:          piper / nero / piper_h / piper_l / piper_x
    #   can_port:          CAN 接口名，默认 can0
    #   auto_enable:       启动时自动使能关节（默认 true）
    #   enable_timeout:    使能/固件等待超时秒数（默认 5.0）
    #   fast_mode:         关节控制走 move_js(MIT) 模式（默认 false）
    #   speed_percent:     运动速度百分比（默认 100）
    #   effector_type:     none / agx_gripper / revo2 / revo2_touch
    #   tcp_offset:        TCP 偏移 [x,y,z,roll,pitch,yaw]
    #   control_enabled:   是否接受外部控制指令（默认 true）
    #   pub_rate:          反馈发布频率 Hz（默认 200）
    #
    # 注意：控制器初始化时会读固件版本，
    # 如果 CAN 不通或机械臂未上电，进程会 exit(1) 退出。
    # ============================================================
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_arm_ctrl'),
                'launch', 'start_single_agx_arm.launch.py',
            ])
        ]),
        launch_arguments={
            'arm_type': 'piper',
            'can_port': 'can0',
        }.items(),
    )

    # ============================================================
    # 3. robot_state_publisher + joint_state_publisher_gui
    #    加载完整底盘+机械臂 URDF，发布 TF 链
    #    joint_state_publisher_gui 提供滑块控制关节
    # ============================================================
    model_path = '/home/ros/MC_ARM_baseline_nav2_ok/src/driver/car_sim/urdf/car_with_piper.urdf.xacro'
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', 'control/joint_states')],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'rate': 200}],
        remappings=[('joint_states', 'control/joint_states')],
    )

    # ============================================================
    # 4. 静态 TF: link6 → camera_link
    #    手眼标定的初始估计值，后续由 easy_handeye 标定优化
    # ============================================================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.1',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'link6', '--child-frame-id', 'camera_link',
        ],
    )

    # ============================================================
    # 5. rviz2（显示 Camera 图像 + TF 标记帧，无 RobotModel）
    # ============================================================
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'ar_calib.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # ============================================================
    # 6. ArUco 二维码检测
    #    发布 camera_color_optical_frame → camera_marker 的 TF
    # ============================================================
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('piper_d435i_handeye'),
                'launch', 'aruco_single.launch.py',
            ])
        ]),
        launch_arguments={
            'camera_image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }.items(),
    )

    # ============================================================
    # 7. EasyHandEye 手眼标定 GUI
    #    handeye_server + rqt_calibrator
    # ============================================================
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('piper_d435i_handeye'),
                'launch', 'handeye_calibrate.launch.py',
            ])
        ]),
    )

    return LaunchDescription([
        # 1
        camera_launch,
        # 2
        arm_launch,
        # 3
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # 4
        static_tf_node,
        # 5 rviz: Camera image + TF marker, no RobotModel
        rviz_node,
        # 6
        aruco_launch,
        # 7
        calibration_launch,
    ])
