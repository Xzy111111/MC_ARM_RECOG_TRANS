# pointcloud_to_laserscan

ROS2 点云 → 激光扫描转换器。将 3D 点云投影为 2D LaserScan，使 Nav2 等 2D 算法能使用 3D 雷达数据。

---

## 在本项目中的作用

```
Livox mid360（3D 雷达）
    ↓ 点云话题 /livox/lidar（PointCloud2）
pointcloud_to_laserscan
    ↓ LaserScan 话题 /livox/scan
Nav2 导航栈（AMCL / costmap / MPPI）
```

mid360 是 360° 水平视场角 + 90° 垂直视场角的 3D 雷达。Nav2 的 2D costmap 需要 LaserScan 格式的输入，所以需要这个转换桥接。

---

## 配置说明

配置文件：`config/params_pointcloud_to_laserscan.yaml`

**当前配置（适配 Livox mid360 + 麦轮底盘 + Piper 臂）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `cloud_in_topic` | `/livox/lidar` | mid360 点云输入 |
| `scan_out_topic` | `/livox/scan` | 转换后的激光扫描输出 |
| `target_frame` | `base_link` | 投影到机器人坐标系 |
| `min_height` | 0.0 | 底部截止（地面以下不取） |
| `max_height` | 0.5 | 顶部截止（避开机械臂遮挡） |
| `angle_min / angle_max` | ±π | 360° 全周扫描 |
| `range_min / range_max` | 0.2 / 15.0 m | 有效距离范围 |

**注意：** `max_height: 0.5` 将高于 0.5m 的点云过滤掉，避免机械臂自身被 Nav2 识别为障碍物。如果机械臂抬升导致原地规划无解，可以调高此值或改为双扫描。

---

## 双扫描功能

支持同时输出两路不同高度范围的 LaserScan：

| 扫描 | 配置前缀 | 默认启用 | 用途 |
|------|----------|----------|------|
| Scan 1 | （主参数） | 是 | 地面障碍物（0.0~0.5m） |
| Scan 2 | `*2` 后缀参数 | 否 | 高空障碍物（0.4~0.5m，可选） |

---

## 启动

已在 `bringup.launch.py` 中集成，无需单独启动。

```bash
ros2 launch bringup bringup.launch.py
```

也可单独启动：

```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan.launch.py
```

---

## 文件结构

```
pointcloud_to_laserscan/
├── config/
│   └── params_pointcloud_to_laserscan.yaml   项目配置
├── launch/
│   ├── pointcloud_to_laserscan.launch.py     转换节点启动
│   ├── sample_pointcloud_to_laserscan_launch.py  示例
│   └── sample_laserscan_to_pointcloud_launch.py  反向转换示例
├── src/
│   ├── pointcloud_to_laserscan_node.cpp      点云→LaserScan
│   ├── laserscan_to_pointcloud_node.cpp      LaserScan→点云
│   └── dummy_pointcloud_publisher.cpp        测试用伪点云发布器
└── include/
    └── pointcloud_to_laserscan/
        └── ...
```
