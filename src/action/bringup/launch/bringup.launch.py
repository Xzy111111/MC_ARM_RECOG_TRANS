import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mid360_driver'),
                         'launch', 'mid360_driver.launch.py')
        ),
        launch_arguments={'use_rviz': 'true'}.items()
    )

    vehicle_driver = ExecuteProcess(
        cmd=['ros2', 'run', 'vehicle_driver', 'vehicle_driver'],
        output='screen'
    )

    small_point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('small_point_lio'),
                         'launch', 'small_point_lio.launch.py')
        )
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                   "--frame-id", "base_link", "--child-frame-id", "livox_frame"],
    )

    pcl_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                         'launch', 'pointcloud_to_laserscan.launch.py')
        )
    )

    config_slam = os.path.join(
        get_package_share_directory('bringup'),
        'config', 'slamtoolbox.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': config_slam,
        }.items()
    )

    config_nav2 = os.path.join(
        get_package_share_directory('bringup'),
        'config', 'default.yaml')
    empty_map = os.path.join(
        get_package_share_directory('bringup'),
        'config', 'empty_map.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': config_nav2,
            'map': empty_map,
        }.items()
    )

    return LaunchDescription([
        mid360_launch,
        vehicle_driver,
        small_point_lio_launch,
        lidar_static_tf,
        pcl_to_scan,
        slam_toolbox,
        nav2,
    ])
