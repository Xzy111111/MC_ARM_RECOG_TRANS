from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    启动 easy_handeye2 手眼标定 GUI。
    在 rqt/Rviz 中采集 20 组机械臂不同位姿下的二维码数据，
    自动计算相机与机械臂末端之间的位姿变换矩阵。

    标定模式：eye_in_hand（眼在手上）
      相机固定在机械臂末端法兰上，随机械臂运动。
      最终输出：camera_link 相对于 flange_link 的变换矩阵。

    坐标系说明（适配 Piper 六轴机械臂 + D435i）：
      - robot_base_frame:      base_link              机械臂基座
      - robot_effector_frame:  flange_link            末端法兰，camera 安装处
      - tracking_base_frame:   camera_link            D435i 主体坐标系
      - tracking_marker_frame: camera_marker          ArUco 二维码坐标系
    """

    # 标定类型声明
    calibration_type_arg = DeclareLaunchArgument(
        "calibration_type",
        default_value="eye_in_hand",
        choices=["eye_in_hand", "eye_on_base"],
        description="标定类型：eye_in_hand (眼在手上) / eye_on_base (眼在基座)",
    )

    # 标定名称 (用于保存/加载历史标定结果)
    name_arg = DeclareLaunchArgument(
        "name",
        default_value="piper_d435i_eih",
        description="标定任务唯一标识，用于存储/加载历史数据",
    )

    # 包含 easy_handeye2 的标定 Launch 文件
    handeye_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("easy_handeye2"),
                "launch",
                "calibrate.launch.py",
            ])
        ]),
        launch_arguments={
            "calibration_type": LaunchConfiguration("calibration_type"),
            "name": LaunchConfiguration("name"),

            # ---- Piper 六轴机械臂坐标系 ----
            "robot_base_frame": "base_link",
            "robot_effector_frame": "link6",
            # Piper 臂最后一个连杆是 link6 (非标法兰)

            # ---- D435i 相机 & ArUco 二维码坐标系 ----
            "tracking_base_frame": "camera_link",
            "tracking_marker_frame": "camera_marker",

            # ---- 标定行为 ----
            "freehand_robot_movement": "true",

            # ---- 可选 MoveIt 配置（若有 move_group 可用） ----
            # "move_group_namespace": "/",
            # "move_group": "manipulator",
        }.items(),
    )

    return LaunchDescription([
        calibration_type_arg,
        name_arg,
        handeye_calibration,
    ])
