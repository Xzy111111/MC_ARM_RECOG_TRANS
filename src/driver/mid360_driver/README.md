# Mid-360 驱动

这是一个 Mid-360 驱动的实现，旨在作为 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 的替代方案。

## 特性

- 不依赖 Livox-SDK2，而是直接实现 UDP 通信，因此非常轻量
- 支持自动获取雷达 IP，无需手动配置雷达 IP
- 支持多雷达
- 在未启用时间同步时，只会在收到第一帧数据时计算一次时间差并在后续使用该时间差，以保证基本的时间同步
- 支持点云坐标变换（旋转和平移）
- 支持点云距离筛选和轴向筛选
- 支持 IMU 低通滤波

## 特别说明

发布的点云格式为 PointCloud2，与 livox_ros_driver2 的点云格式不同，用户可能需要修改其他包中的代码。

## 安装依赖

1. 请确保已经安装 ROS2
2. 安装 Asio。如果你使用的是 Ubuntu，可以通过以下命令安装：
   `sudo apt install libasio-dev`

## 参数说明

以下是在配置文件中可设置的参数：

```yaml
mid360_driver:
    ros__parameters:
        # 基础话题配置
        lidar_topic: /livox/lidar
        imu_topic: /livox/imu
        lidar_frame: livox_frame
        imu_frame: imu_frame
        lidar_publish_time_interval: 0.1  # 点云发布频率 (10Hz)
        host_ip: 192.168.32.81  # 当前主机ip

        # TF 变换配置
        publish_tf: true        # 是否发布 TF 变换
        transform_enable: true  # 是否启用坐标变换

        # 点云坐标变换参数
        rotation_roll: 0.0      # 绕 X 轴旋转角度（度）
        rotation_pitch: 0.0     # 绕 Y 轴旋转角度（度）
        rotation_yaw: 0.0       # 绕 Z 轴旋转角度（度）
        translation_x: 0.0      # X 轴平移（米）
        translation_y: 0.0      # Y 轴平移（米）
        translation_z: 0.0      # Z 轴平移（米）

        # IMU 低通滤波参数
        imu_filter_enable: false           # 是否启用 IMU 低通滤波
        imu_filtered_topic: /livox/imu_filtered   # 滤波后 IMU 话题名
        imu_filter_alpha: 0.2              # 一阶低通滤波系数 (0 < alpha <= 1)
                                           # alpha 越小，滤波越强（但延迟越大）
                                           # 推荐：静止抖动严重时 0.05~0.2，运动时 0.5~0.9

        # 点云距离筛选参数
        min_point_distance: 0.03  # 保留 >= 该距离的点云（米），-1 表示不启用
        max_point_distance: -1.0  # 保留 <= 该距离的点云（米），-1 表示不启用

        # 点云轴向筛选参数
        enable_x_filter: false  # 是否启用 X 轴筛选（前后方向）
        min_x: -1.0             # 保留 >= 该 X 坐标的点云（米）
        max_x: 1.0              # 保留 <= 该 X 坐标的点云（米）
        enable_y_filter: false  # 是否启用 Y 轴筛选（左右方向）
        min_y: -1.0             # 保留 >= 该 Y 坐标的点云（米）
        max_y: 1.0              # 保留 <= 该 Y 坐标的点云（米）
        enable_z_filter: false  # 是否启用 Z 轴筛选（高度方向）
        min_z: -1.0             # 保留 >= 该 Z 坐标的点云（米）
        max_z: 1.0              # 保留 <= 该 Z 坐标的点云（米）
```

## 使用示例

### 启动驱动

```bash
ros2 launch mid360_driver mid360_driver.launch.py
```
