from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_small_point_lio = PathJoinSubstitution(
        [FindPackageShare("small_point_lio"), "config", "mid360.yaml"]
    )
    action_small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[config_path_small_point_lio],
    )
    return LaunchDescription([action_small_point_lio])
