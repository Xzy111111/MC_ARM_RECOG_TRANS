#!/usr/bin/env bash
# 启动 Realsense D435i 相机驱动 (供 exts/script 下的标定脚本自动调用)
# 用法: ./launch_camera.sh
set -e

# 自动找到可用的 ROS 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
# 如果用户有自己的工作空间环境, 让其自动加载
for ws in ~/mc_robot_ws/install/setup.bash ~/MC_ARM_baseline_nav2_ok/install/setup.bash; do
    if [ -f "$ws" ]; then
        # shellcheck disable=SC1091
        source "$ws"
    fi
done

exec ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    pointcloud.enable:=false \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30 \
    enable_sync:=true \
    align_depth.enable:=false \
    decimation_filter.enable:=true
