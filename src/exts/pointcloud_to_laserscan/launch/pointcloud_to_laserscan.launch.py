import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('pointcloud_to_laserscan'),
        'config',
        'params_pointcloud_to_laserscan.yaml'
    )   
    # 加载并解析 YAML 文件
    with open(config_dir, 'r') as f:
        params = yaml.safe_load(f)
        cloud_in_topic = params['pointcloud_to_laserscan']['ros__parameters']['cloud_in_topic']
        scan_out_topic = params['pointcloud_to_laserscan']['ros__parameters']['scan_out_topic']
    
    action_pointcloud_to_laserscan = launch_ros.actions.Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  [cloud_in_topic]),
                    ('scan',  [scan_out_topic])],
        parameters=[config_dir],
        name='pointcloud_to_laserscan'
    )
    return LaunchDescription([
        action_pointcloud_to_laserscan,
    ])

