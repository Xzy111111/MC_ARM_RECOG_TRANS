# calibration

手眼标定：计算相机与机械臂末端之间的固定变换矩阵。

---

## 方案

采用 **eye-in-hand（眼在手上）**：D435i 相机固定在 Piper 臂末端法兰上，随机械臂运动，标定板固定放置。

**标定流程：**

```
1. 移动机械臂到不同位姿（每次改变姿态角）
2. easy_handeye2 在每个位姿记录一组样本：
   ├── base_link → link6    （来自 robot_state_publisher）
   └── camera_link → camera_marker （来自 aruco_ros）
3. 采集 5~20 组样本后，调用 OpenCV calibrateHandEye() 解算
4. 输出：link6 → camera_link 变换矩阵
```

---

## 包结构

```
calibration/
├── piper_d435i_handeye/          ← 集成包（针对 Piper + D435i）
│   ├── launch/
│   │   ├── aruco_single.launch.py       检测 ArUco 二维码
│   │   ├── handeye_calibrate.launch.py  启动标定 GUI（采样 + 计算）
│   │   ├── check_evaluate.launch.py     评估标定精度
│   │   └── publish.launch.py            发布已保存的标定结果
│   ├── config/
│   │   └── calibration_params.yaml      标定参数配置
│   ├── rviz/
│   │   └── handeye.rviz                 标定 rviz 配置
│   └── package.xml
│
└── realsense_ws/src/             ← 外部依赖源码
    ├── aruco_ros/                ArUco 二维码检测 ROS 包装
    │   ├── aruco/                ArUco 检测库核心 (v3.0.4)
    │   ├── aruco_msgs/           ROS 消息定义
    │   └── aruco_ros/            ROS 节点（single/double/marker_publish）
    └── easy_handeye2/            手眼标定求解器（ROS2 移植版）
        ├── handeye_server.py     标定服务端（采样 + 计算 + 保存）
        ├── handeye_sampler.py    TF 采样管理
        ├── handeye_calibration_backend_opencv.py  OpenCV 求解
        ├── handeye_client.py     客户端接口
        └── handeye_rqt_calibrator/scripts.py  标定 GUI
```

---

## 启动

通过 `AR_bringup.launch.py` 一键启动（推荐）：

```bash
ros2 launch bringup AR_bringup.launch.py
```

也可单独启动各组件：

```bash
# 仅启动 ArUco 检测
ros2 launch piper_d435i_handeye aruco_single.launch.py

# 仅启动标定 GUI
ros2 launch piper_d435i_handeye handeye_calibrate.launch.py

# 发布已保存的标定结果
ros2 launch piper_d435i_handeye publish.launch.py

# 评估标定精度
ros2 launch piper_d435i_handeye check_evaluate.launch.py
```

---

## 标定参数

配置文件：`piper_d435i_handeye/config/calibration_params.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| type | `eye_in_hand` | 相机固定在臂末端 |
| name | `piper_d435i_eih` | 标定标识 |
| robot_base_frame | `base_link` | 臂基座坐标系 |
| robot_effector_frame | `link6` | 臂末端（相机安装处） |
| tracking_base_frame | `camera_link` | 相机主体 |
| tracking_marker_frame | `camera_marker` | 标定板 |

---

## 依赖关系

```
相机图像 (/camera/camera/color/image_raw)
    ↓
aruco_ros (single)
    ├── 检测 ArUco 二维码
    ├── 发布 camera_color_optical_frame → camera_marker TF
    └── 发布 /aruco_single/pose
        ↓
easy_handeye2
    ├── handeye_server (服务端)
    │   ├── 检查 TF 连通性
    │   ├── 采样 base_link→link6 + camera_link→camera_marker
    │   └── 调用 OpenCV 计算标定结果
    ├── handeye_rqt_calibrator (GUI)
    │   └── [Take Sample] / [Compute] / [Save] 按钮
    └── handeye_publisher
        └── 发布标定结果 link6→camera_link TF
```
