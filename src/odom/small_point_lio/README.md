# Small Point-LIO

Small Point-LIO 是 [Point-LIO 算法](https://github.com/hku-mars/Point-LIO) 的高级实现，相比原始版本提供 2-3 倍的速度提升。

如果想了解为什么速度这么快，请阅读[这篇文章](https://bbs.robomaster.com/article/813022)。


## 参数配置

以下是可以在配置文件中设置的参数：

```yaml
small_point_lio:
    ros__parameters:
        lidar_topic: /livox/lidar                  # 点云话题名
        imu_topic: /livox/imu                      # IMU话题名
        lidar_type: custom_mid360_driver              # 雷达类型 livox_custom_msg, livox_pointcloud2, custom_mid360_driver
        lidar_frame: livox_frame                   # 雷达坐标系
        save_pcd: true                            # 是否保存点云

        # 点云过滤
        point_filter_num: 1                        # 多少个点取一个点
        min_distance: 0.5                          # 点云半径最小值，小于该值会被过滤
        max_distance: 1000.0                       # 点云半径最大值，大于该值会被过滤
        space_downsample: true                     # 是否启用点云降采样
        space_downsample_leaf_size: 0.2            # 点云降采样时使用的栅格大小

        # IMU处理
        gravity: [0.0, 0.0, -9.810]                # 重力
        fix_gravity_direction: true                # 是否收集前200个IMU数据修正重力方向，重力的大小依旧从gravity参数获得
        check_satu: true                           # 是否开启IMU的数据饱和检查
        satu_acc: 3.0                              # IMU的加速度饱和阈值
        satu_gyro: 35.0                            # IMU的角速度饱和阈值
        acc_norm: 1.0                              # IMU的加速度模长

        # 地图
        map_resolution: 0.3                        # 地图分辨率
        init_map_size: 10                          # 接收到多少个点才初始化地图

        # 雷达与IMU相对位姿
        extrinsic_est_en: false                    # 雷达与IMU的相对位姿是否通过预测得到
        extrinsic_T: [-0.011, -0.02329, 0.04412]
        extrinsic_R: [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]

        # 滤波器参数
        # R
        laser_point_cov: 0.01                      # 激光点的协方差
        imu_meas_acc_cov: 0.01                     # IMU测量的加速度协方差
        imu_meas_omg_cov: 0.01                     # IMU测量的角速度协方差
        # Q
        velocity_cov: 20.0                         # 速度的协方差
        acceleration_cov: 500.0                    # 加速度的协方差
        omg_cov: 1000.0                            # 角速度的协方差
        ba_cov: 0.0001                             # 加速度偏置的协方差
        bg_cov: 0.0001                             # 角速度偏置的协方差
        plane_threshold: 0.1                       # 平面匹配阈值，越小表示越严格
        match_sqaured: 81.0                        # 当前点是否在平面上的阈值，越小表示越严格

        # 数据发布
        publish_odometry_without_downsample: false # 是否发布高频的里程计
```

## 保存地图

**步骤 1**：在配置文件中将 `save_pcd` 设置为 `true`。

**步骤 2**：运行 small_point_lio 直到地图构建完成。

**步骤 3**：通过调用服务保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

> 注意：请确保有足够的内存来保存地图。保存完成后，不要忘记将 `save_pcd` 设置为 `false`。

