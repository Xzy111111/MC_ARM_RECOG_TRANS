#!/usr/bin/env python3
# -*-coding:utf8-*-
"""Piper 机械臂 MoveIt 一站式启动。

规划坐标系 = base_link（与导航 map->odom->base_link 无缝衔接，TF 唯一无冲突）。

用法:
    # 纯仿真 demo（mock 硬件，无需真机/CAN）
    ros2 launch piper_control piper_moveit.launch.py use_rviz:=true follow:=false

    # 真机（需 CAN 上电）
    sudo ip link set can0 up type can bitrate 1000000
    ros2 launch piper_control piper_moveit.launch.py use_rviz:=true use_hardware:=true

数据流（真机模式）:
    move_group → arm_controller(JointTrajectoryController) → mock 硬件
    → joint_state_broadcaster → /control/joint_states → agx_arm_ctrl → move_j 真机
    move_group 订阅 /feedback/joint_states（agx_arm_ctrl 发布的真实关节状态）
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config(use_sensors=True):
    builder = (
        MoveItConfigsBuilder("piper_arm", package_name="piper_control")
        .robot_description(file_path="urdf/piper_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/piper.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
    )
    if use_sensors:
        builder = builder.sensors_3d(file_path="config/sensors_3d.yaml")
    config = builder.to_moveit_configs()
    if not use_sensors:
        # to_moveit_configs() 会自动注入默认 sensors_3d；显式移除避免加载 octomap
        config.sensors_3d = {}
    return config


def _joint_states_topic(context):
    """follow:=true → feedback (真实状态); false → control (mock 回读)"""
    follow = LaunchConfiguration("follow").perform(context)
    if follow == "true":
        return LaunchConfiguration("feedback_topic").perform(context)
    return LaunchConfiguration("control_topic").perform(context)


def _launch_all(context):
    use_sensors = LaunchConfiguration("use_sensors").perform(context) == "true"
    moveit_config = _moveit_config(use_sensors=use_sensors)
    js_topic = _joint_states_topic(context)

    joint_states_remap = [("joint_states", js_topic)]
    # robot_description 全局 remap 与官方一致，避免参数名字被 PushRosNamespace 改写
    robot_description_remap = [("/robot_description", "robot_description")]

    move_group_config = {
        "planning_frame": "base_link",
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    nodes = []

    # robot_state_publisher：发布 base_link→link6→tcp_link TF
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
            remappings=joint_states_remap + robot_description_remap,
        )
    )

    # move_group
    nodes.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                move_group_config,
            ],
            remappings=joint_states_remap + robot_description_remap,
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        )
    )

    # ros2_control（mock 硬件 + arm_controller + joint_state_broadcaster）
    ros2_controllers_yaml = os.path.join(
        FindPackageShare("piper_control").perform(context),
        "config", "ros2_controllers.yaml",
    )
    nodes.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_yaml,
            ],
            remappings=[("joint_states", LaunchConfiguration("control_topic").perform(context))]
            + robot_description_remap,
        )
    )

    # spawn arm_controller + joint_state_broadcaster
    for controller in ["arm_controller", "joint_state_broadcaster"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # 可选: 真机驱动 agx_arm_ctrl（订阅 /control/joint_states，发布 /feedback/joint_states）
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("agx_arm_ctrl"),
                    "launch", "start_single_agx_arm.launch.py",
                ])
            ),
            launch_arguments={
                "arm_type": LaunchConfiguration("arm_type"),
                "effector_type": LaunchConfiguration("effector_type"),
                "can_port": LaunchConfiguration("can_port"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "speed_percent": LaunchConfiguration("speed_percent"),
                "tcp_offset": LaunchConfiguration("tcp_offset"),
                "control_enabled": LaunchConfiguration("hardware_control_enabled"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_hardware")),
        )
    )

    # 可选: 执行期控制门控（防 RViz 滑块等外部源经 /control/joint_states 干扰）
    nodes.append(
        Node(
            package="agx_arm_moveit",
            executable="agx_arm_control_gate",
            output="screen",
            parameters=[{
                "status_topics": [
                    "arm_controller/follow_joint_trajectory/_action/status",
                ],
                "gate_service_name": LaunchConfiguration("control_gate_service"),
            }],
            condition=IfCondition(LaunchConfiguration("auto_control_gate")),
        )
    )

    # 可选: RViz
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if not rviz_config:
        rviz_config = os.path.join(
            FindPackageShare("piper_control").perform(context),
            "config", "moveit.rviz",
        )
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            remappings=joint_states_remap + robot_description_remap,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return nodes


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "use_sensors", default_value="true",
            description="Load octomap point cloud sensor (needs camera TF chain). "
                        "Disable if no camera / to avoid octomap blocking planning.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description="RViz config file path (empty = default MoveIt config)",
        ),
        DeclareLaunchArgument(
            "follow", default_value="true", choices=["true", "false"],
            description="Follow real arm state. true: subscribe feedback/joint_states "
                        "(real arm); false: subscribe control/joint_states (mock demo).",
        ),
        DeclareLaunchArgument(
            "feedback_topic", default_value="feedback/joint_states",
            description="Joint states feedback topic (follow:=true)",
        ),
        DeclareLaunchArgument(
            "control_topic", default_value="control/joint_states",
            description="Joint states control topic (follow:=false, and ros2_control remap)",
        ),
        DeclareLaunchArgument(
            "use_hardware", default_value="false",
            description="Launch real arm driver (agx_arm_ctrl) via CAN",
        ),
        DeclareLaunchArgument(
            "arm_type", default_value="piper",
            description="Arm type (piper/piper_x/piper_l/piper_h/nero)",
        ),
        DeclareLaunchArgument(
            "effector_type", default_value="none",
            choices=["none", "agx_gripper", "revo2", "revo2_touch"],
            description="End effector type. agx_gripper initializes the Piper gripper.",
        ),
        DeclareLaunchArgument(
            "can_port", default_value="can0",
            description="CAN port for the real arm",
        ),
        DeclareLaunchArgument(
            "auto_enable", default_value="true", choices=["true", "false"],
            description="Automatically enable the arm on startup",
        ),
        DeclareLaunchArgument(
            "speed_percent", default_value="100",
            description="Arm movement speed percent (0-100)",
        ),
        DeclareLaunchArgument(
            "tcp_offset", default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="TCP offset [x, y, z, roll, pitch, yaw]",
        ),
        DeclareLaunchArgument(
            "hardware_control_enabled", default_value="true",
            description="Whether agx_arm_ctrl accepts external control commands",
        ),
        DeclareLaunchArgument(
            "auto_control_gate", default_value="false", choices=["true", "false"],
            description="Gate /control commands during MoveIt execution only",
        ),
        DeclareLaunchArgument(
            "control_gate_service", default_value="control_enable",
            description="SetBool gate service for agx_arm_control_gate",
        ),
    ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=_launch_all)]
    )
