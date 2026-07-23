from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    启动 ArUco 二维码检测节点，读取相机图像并识别标定板上的二维码。

    坐标系说明：
      - reference_frame / camera_frame -> camera_color_optical_frame
        (相机光心坐标系，由 realsense-ros 发布)
      - marker_frame -> camera_marker
        (二维码坐标系，由 aruco_ros 发布)

    话题 remapping（适配 Realsense D435i 默认话题名）：
      /camera_info  <-  /camera/color/camera_info
      /image        <-  /camera/color/image_raw
    """

    aruco_single_params = {
        "image_is_rectified": True,
        "marker_size": LaunchConfiguration("marker_size"),
        "marker_id": LaunchConfiguration("marker_id"),
        "reference_frame": LaunchConfiguration("reference_frame"),
        "camera_frame": LaunchConfiguration("reference_frame"),  # 与 reference_frame 相同
        "marker_frame": LaunchConfiguration("marker_frame"),
        "corner_refinement": LaunchConfiguration("corner_refinement"),
    }

    aruco_single = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_single",
        parameters=[aruco_single_params],
        remappings=[
            ("/camera_info", LaunchConfiguration("camera_info_topic")),
            ("/image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    return [aruco_single]


def generate_launch_description():
    # ---- ArUco 标定板参数 ----
    marker_id_arg = DeclareLaunchArgument(
        "marker_id",
        default_value="6",
        description="ArUco 二维码 ID，需与打印的标定板一致",
    )

    marker_size_arg = DeclareLaunchArgument(
        "marker_size",
        default_value="0.1",
        description="ArUco 二维码边长 (单位: 米)，例如 100mm → 0.1",
    )

    reference_frame_arg = DeclareLaunchArgument(
        "reference_frame",
        default_value="camera_color_optical_frame",
        description="相机参考坐标系 (realsense 发布的相机光心坐标系)",
    )

    marker_frame_arg = DeclareLaunchArgument(
        "marker_frame",
        default_value="camera_marker",
        description="二维码在 TF 中的坐标系名称",
    )

    corner_refinement_arg = DeclareLaunchArgument(
        "corner_refinement",
        default_value="LINES",
        choices=["NONE", "HARRIS", "LINES", "SUBPIX"],
        description="角点优化方法",
    )

    # ---- Realsense D435i 相机话题 ----
    camera_info_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="D435i 彩色相机信息话题",
    )

    camera_image_arg = DeclareLaunchArgument(
        "camera_image_topic",
        default_value="/camera/color/image_raw",
        description="D435i 彩色图像话题",
    )

    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(corner_refinement_arg)
    ld.add_action(camera_info_arg)
    ld.add_action(camera_image_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
