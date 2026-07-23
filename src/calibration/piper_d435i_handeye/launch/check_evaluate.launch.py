from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    启动 easy_handeye2 标定精度评估界面。

    在完成首次标定后运行，可：
      1. 采集多组新样本用于验证标定精度
      2. 可视化标定误差分布
      3. 若采集足够样本，可进行二次标定优化

    注意：需在首次标定完成并保存结果后运行。
    """

    name_arg = DeclareLaunchArgument(
        "name",
        default_value="piper_d435i_eih",
        description="标定任务标识，需与 handeye_calibrate.launch.py 中一致",
    )

    handeye_evaluate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("easy_handeye2"),
                "launch",
                "evaluate.launch.py",
            ])
        ]),
        launch_arguments={
            "name": LaunchConfiguration("name"),
        }.items(),
    )

    return LaunchDescription([
        name_arg,
        handeye_evaluate,
    ])
