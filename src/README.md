# MC_ARM_baseline_nav2_ok

麦轮底盘 + Piper 六轴机械臂 + D435i 相机 + Livox mid360 激光雷达的移动操作机器人原型。

---

## 硬件平台

| 组件 | 型号 | 通信接口 | 协议 |
|------|------|----------|------|
| **底盘** | 麦克纳姆轮（麦轮）四驱底盘 | 串口 `/dev/ttyACM0`, 2Mbps | 自定义 24 字节帧协议（vx/vy/vw 编码） |
| **机械臂** | AgileX Piper 六轴 | CAN bus `can0`, 1Mbps | Piper CAN 协议（CAN 2.0 → CAN FD 隧道） |
| **相机** | Intel RealSense D435i | USB 3.0 | ROS2 + librealsense |
| **激光雷达** | Livox mid360 | USB/以太网 | Livox SDK → pointcloud |

**底盘通信协议（vehicle_driver 串口协议）：**

```
帧头(0xAA) | vx_i16 | vy_i16 | vw_i16 | 使能区 | 保留 | 反馈区 | 帧尾(0x55)
```
订阅 `/cmd_vel`，以 100Hz 发送到 STM32F407 驱动麦轮电机。

**机械臂通信协议（Piper CAN 协议）：**

基于 python-can (socketcan)，CAN 2.0 标准帧承载 CAN FD 隧道，0x711 请求 / 0x712 响应。通过 `agx_arm_ctrl_single` 节点经由 CAN bus 发送关节角度/笛卡尔控制指令。

---

## 软件架构

```
src/
├── action/bringup/           ← 主启动 + 导航配置
│   ├── launch/AR_bringup.launch.py   一键启动（相机 + 臂 + GUI）
│   ├── launch/bringup.launch.py      Nav2 导航启动
│   └── config/               AMCL / MPPI / costmap 配置
├── calibration/              手眼标定
│   ├── piper_d435i_handeye/  标定参数 + 适配 launch
│   └── realsense_ws/src/
│       ├── aruco_ros/        ArUco 二维码检测
│       └── easy_handeye2/    手眼标定求解器（Tsai-Lenz 等）
├── driver/                   硬件驱动
│   ├── car_sim/              底盘 URDF 模型
│   ├── vehicle_driver/       底盘串口驱动（/cmd_vel → 串口）
│   └── piper_driver/         Piper 机械臂驱动（CAN）
├── odom/                     里程计
│   └── small_point_lio/      LiDAR-IMU 紧耦合里程计
└── exts/                     工具（非 ROS 包）
    ├── pointcloud_to_laserscan/  Livox 点云转 LaserScan
    └── script/               手眼标定 & ArUco 工具脚本（详见其 README）
        ├── calib_data_collector.py   标定数据自动采集器 → data/calib_data.csv
        ├── calib_compute_handeye.py  手眼标定求解（正运动学 + calibrateHandEye）
        └── rgb_viewer.py             ArUco 检测显示 + 发布 camera_marker TF
```

---

## 当前状态

| 功能 | 状态 | 说明 |
|------|------|------|
| 底盘控制 | ✅ | 串口通信，键盘可遥控，接收 `/cmd_vel` |
| 机械臂 CAN 控制 | ✅ | 固件 S-V1.8-8，使能正常，`joint_state_publisher_gui` 可操控 |
| 相机图像 | ✅ | D435i 实时 1280×720，rviz 显示 |
| ArUco 标定板检测 | ✅ | `exts/script/rgb_viewer.py` 识别 id=12(0.128m),发布 `camera_marker` TF |
| 手眼标定 | ✅ | `exts/script/` 自采集 171 组数据求解,静态 TF 已写入 launch |
| 激光雷达点云 | ✅ | Livox mid360 |
| LiDAR-IMU 里程计 | ✅ | small_point_lio |
| Nav2 导航 | ✅ | MPPI 控制器，配置已完成 |

| 待实现功能 | 状态 | 说明 |
|-----------|------|------|
| **夹取（抓取）** | ⬜ | 需要规划：目标检测 → 接近 → 夹爪控制 → 闭合 |
| **拾取（吸取）** | ⬜ | 如果有吸盘末端，需气路控制和位姿对准 |
| **搬运** | ⬜ | 夹取/拾取后保持物体，移动底盘到目标点 |
| **移动到指定位置** | ⬜ | 集成导航：接收目标点 → Nav2 规划 → 底盘跟踪 → 避障 |
| **完整抓取-搬运流水线** | ⬜ | 将以上串联为一个完整动作序列 |

---

## 启动方式

```bash
# 一键启动（相机 + 臂 + 标定 GUI）
ros2 launch bringup AR_bringup.launch.py

# 启动导航（需先启动底盘驱动 + 里程计）
ros2 launch bringup bringup.launch.py

# 底盘键盘遥控
ros2 run vehicle_driver keyboard_test
```

## 手眼标定脚本工具

`piper_d435i_handeye` 包已删除,标定链路改由 `exts/script/` 下的独立脚本完成(无需构建):

| 步骤 | 脚本 | 作用 | 输出 |
|------|------|------|------|
| ① 采集 | `calib_data_collector.py` | 相机画面 + ArUco 检测,检测到 id=12 自动记录 6 关节角 + tag 位姿 | `data/calib_data.csv` |
| ② 求解 | `calib_compute_handeye.py` | 正运动学 + `calibrateHandEye`,解出 `link6_T_camera` | `data/calib_result.txt` |
| ③ 发布 | `rgb_viewer.py` | 实时检测显示,发布 `camera_color_optical_frame → camera_marker` TF | 动态 TF |

求解结果以静态 TF 形式固化在 `AR_bringup.launch.py`(`link6 → camera_link`,2026-07-30 标定)。

```bash
cd src/exts/script
python3 calib_data_collector.py    # ① 摆多个机械臂姿态,自动采集
python3 calib_compute_handeye.py   # ② 离线求解(可加 CSV 路径或 --list)
python3 rgb_viewer.py              # ③ 实时检测 + TF(按 Q 退出)
```

## 启动后出现

| 窗口 | 功能 |
|------|------|
| rviz2 | 相机画面 + TF 显示 |
| joint_state_publisher_gui | 机械臂关节滑块 |
| handeye_rqt_calibrator | 手眼标定采样界面 |

## 待实现功能架构（规划）

```
接收目标位置（例如 "去桌子前抓杯子"）
    ↓
Nav2 导航 → 移动到指定位置（含避障）
    ↓
视觉检测 → 识别目标物体位置
    ↓
机械臂运动学解算 → 接近目标
    ↓
夹爪/吸盘控制 → 夹取/拾取
    ↓
保持物体 → Nav2 导航到目标放置点
    ↓
释放物体 → 完成搬运
```
