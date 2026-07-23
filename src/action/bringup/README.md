# bringup

一键启动包。包含两套独立的启动流程：

---

## 1. AR_bringup.launch.py —— 手眼标定启动

启动 Piper 机械臂 + D435i 相机的**手眼标定**所需所有节点。

**启动内容：**

| # | 组件 | 说明 |
|---|------|------|
| 1 | Realsense D435i 相机 | 彩色/深度/IMU 数据流 |
| 2 | Piper 机械臂 CAN 控制器 | `agx_arm_ctrl_single`，通过 `can0` 控制 |
| 3 | robot_state_publisher | 加载 `car_with_piper.urdf.xacro`（底盘+臂），发布完整 TF 链 |
| 4 | joint_state_publisher_gui | 关节滑块面板，手动控制每个关节 |
| 5 | 静态 TF `link6 → camera_link` | 手眼标定初始估计（xyz=0 0 0.1） |
| 6 | rviz2 | 显示相机画面 + TF 标记帧，无 RobotModel |
| 7 | ArUco 二维码检测 | 检测 id=6 标定板，发布 `camera_marker` TF |
| 8 | easy_handeye2 标定 GUI | `handeye_server` + `rqt_calibrator` 采样/计算/保存 |

**使用：**
```bash
ros2 launch bringup AR_bringup.launch.py
```

**启动后弹窗：**
- rviz2（相机画面 + TF）
- joint_state_publisher_gui（关节滑块）
- handeye_rqt_calibrator（标定操作窗口）

---

## 2. bringup.launch.py —— 导航启动

启动激光雷达 + 底盘驱动 + LIO 里程计 + Nav2 导航栈。

**启动内容：**

| # | 组件 | 说明 |
|---|------|------|
| 1 | Livox mid360 雷达驱动 | 发布点云 |
| 2 | vehicle_driver 底盘驱动 | 订阅 `/cmd_vel`，串口 2Mbps 控制麦轮 |
| 3 | small_point_lio 里程计 | LiDAR-IMU 紧耦合，发布 `/Odometry` |
| 4 | pointcloud_to_laserscan | mid360 点云转 LaserScan（用于 Nav2） |
| 5 | robot_state_publisher | 发布底盘 TF |
| 6 | Nav2 导航栈 | AMCL 定位 + MPPI 规划/控制 + costmap |
| 7 | rviz2 | Nav2 默认视图 |

**使用：**
```bash
# 先启动底盘驱动
ros2 run vehicle_driver vehicle_driver

# 再启动导航
ros2 launch bringup bringup.launch.py
```

---

## 配置文件

| 文件 | 用途 |
|------|------|
| `config/default.yaml` | Nav2 参数（AMCL / MPPI / costmap / BT） |
| `config/ar_calib.rviz` | 手眼标定 rviz 配置（Camera + TF，无 RobotModel） |
| `config/slamtoolbox.yaml` | SLAM Toolbox 参数（如需建图） |
| `config/empty.pgm` + `.yaml` | 空地图（SLAM 起点） |
| `config/navigate_to_pose_fast_recovery.xml` | 快速恢复行为树 |
| `config/navigate_through_poses_fast_recovery.xml` | 途径点行为树 |
