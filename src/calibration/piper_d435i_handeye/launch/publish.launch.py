from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    将已标定的手眼变换矩阵发布到 ROS2 TF 树。

    标定完成后，使用此 launch 文件持续发布：
      camera_link  <-  flange_link
    的静态变换（即手眼标定结果）。

    在后续运行中，需先启动此发布节点，
    camera_link 与机械臂末端的 TF 关系才能正确建立。
    """

    name_arg = DeclareLaunchArgument(
        "name",
        default_value="piper_d435i_eih",
        description="标定任务标识，需与标定时用的名称一致",
    )

    handeye_publisher = Node(
        package="easy_handeye2",
        executable="handeye_publisher",
        name="handeye_publisher",
        parameters=[{
            "name": LaunchConfiguration("name"),
        }],
    )

    return LaunchDescription([
        name_arg,
        handeye_publisher,
    ])
