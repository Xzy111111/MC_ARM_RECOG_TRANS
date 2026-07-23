import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('car_sim')

    # ===== 启动参数 =====
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='car_with_piper.urdf.xacro',
        description='URDF model file name'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Whether to launch joint_state_publisher_gui'
    )

    model = LaunchConfiguration('model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')

    # ===== robot_state_publisher 发布 URDF 模型 =====
    urdf_path = PathJoinSubstitution([package_dir, 'urdf', model])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ===== joint_state_publisher（默认/不含GUI） =====
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(use_gui)
    )

    # ===== joint_state_publisher_gui 手动调关节（use_gui:=true 时启用） =====
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(use_gui)
    )

    # ===== RViz 可视化 =====
    rviz_config = os.path.join(package_dir, 'config', 'rviz', 'robot_model.rviz')
    rviz = launch.actions.ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        condition=launch.conditions.IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_model,
        declare_use_rviz,
        declare_use_gui,
        robot_state_publisher,
        jsp,
        jsp_gui,
        rviz,
    ])
