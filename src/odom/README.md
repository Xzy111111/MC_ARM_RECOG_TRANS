# odom

LiDAR-IMU 紧耦合里程计，为 Nav2 导航提供高频、低漂移的位姿估计。

---

## 包结构

```
odom/
└── small_point_lio/    LiDAR-IMU 里程计算法
    ├── launch/small_point_lio.launch.py  启动文件
    ├── config/mid360.yaml                Livox mid360 适配配置
    ├── src/                              算法核心（C++）
    ├── include/                          头文件
    └── pcd/                              保存的地图
```

---

## small_point_lio — LiDAR-IMU 里程计算法

基于 [Point-LIO](https://github.com/hku-mars/Point-LIO) 的优化实现，速度提升 2~3 倍。

### 在本项目中的作用

```
Livox mid360（点云 /livox/lidar + IMU /livox/imu）
    ↓
small_point_lio_node（C++，100Hz 以上输出）
    ↓
/Odometry（10Hz，nav_msgs/Odometry）
    ↓
Nav2 导航栈（AMCL + MPPI）
```

mid360 同时输出 3D 点云和 IMU 数据（加速度 + 角速度）。small_point_lio 将两者紧耦合，在无 GPS 环境下提供平滑的里程计，是 Nav2 定位和规划的基石。

### 数据流

| 输入 | 话题 | 说明 |
|------|------|------|
| 3D 点云 | `/livox/lidar` | mid360 点云，custom_mid360_driver 格式 |
| IMU 数据 | `/livox/imu` | 加速度 + 角速度，用于状态预测 |

| 输出 | 话题 | 说明 |
|------|------|------|
| 里程计 | `/Odometry` | 位姿 + 速度，供 Nav2 使用 |
| 点云地图 | 保存到 PCD 文件 | 通过 `/map_save` 服务触发 |

### 关键配置（mid360.yaml）

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | `custom_mid360_driver` | 适配本项目 mid360_driver 的点云格式 |
| `point_filter_num` | 1 | 每 1 个点取 1 个（全采样） |
| `min_distance / max_distance` | 0.5 / 1000.0 m | 有效距离范围 |
| `space_downsample_leaf_size` | 0.2 m | 降采样栅格大小 |
| `extrinsic_T` | `[-0.011, -0.02329, 0.04412]` | 雷达→IMU 平移外参 |
| `map_resolution` | 0.3 m | 地图分辨率 |
| `imu_meas_acc_cov` | 0.01 | IMU 加速度测量协方差 |
| `laser_point_cov` | 0.01 | 激光点协方差 |

### 启动

已在 `bringup.launch.py` 中集成：

```bash
ros2 launch bringup bringup.launch.py
```

也可单独启动：

```bash
ros2 launch small_point_lio small_point_lio.launch.py
```

### 保存地图

```yaml
# 1. 在 config/mid360.yaml 中将 save_pcd 设为 true
save_pcd: true
```

```bash
# 2. 运行建图完成后触发保存
ros2 service call /map_save std_srvs/srv/Trigger
```

保存的 PCD 文件在 `pcd/` 目录下。

---

## 依赖关系

```
Livox mid360
├── /livox/lidar  ──→  small_point_lio  ──→  /Odometry  ──→  Nav2
└── /livox/imu    ──→         ↑
                     LiDAR-IMU 紧耦合
```
