# exts/script — 手眼标定 & ArUco 工具脚本

本目录存放**脱离 ROS 包管理、直接以 `python3` 运行**的标定工具脚本。它们是手眼标定
`calibration/piper_d435i_handeye` 包被删除后的替代方案,完整覆盖从**数据采集 → 求解 →
TF 发布**的整条链路。

> 标定对象: 相机（Intel RealSense D435i）固定在 Piper 机械臂第 6 轴（link6）上,即
> **Eye-on-Hand** 标定。标定板为 ArUco **ID=12**,边长 **0.128 m**。

## 文件说明

| 文件 | 作用 | 依赖 |
|------|------|------|
| [calib_data_collector.py](calib_data_collector.py) | **手眼标定数据采集器（全自动）**。实时显示相机画面并检测 ArUco;检测到目标 ID=12 时,自动读取 `/feedback/joint_states` 的 6 关节角度,连同 tag 位姿一起写入 `data/calib_data.csv`。**自动采集**策略:检测到 Tag + 0.5s 冷却 + 关节变化 > 0.02 rad 才记录一帧 | ROS2, OpenCV, **PySide2** |
| [calib_compute_handeye.py](calib_compute_handeye.py) | **手眼标定求解器**。读取 `calib_data.csv`,用内置 Piper 六轴正运动学(从 `car_with_piper.urdf.xacro` 提取)求 `base_T_link6`,配合 tag 位姿经 `cv2.calibrateHandEye`(Tsai-Lenz 为主,5 种算法对比)解出 `link6_T_camera`。输出 4×4 齐次矩阵 + 可直接用于 `static_transform_publisher` 的命令行参数,并保存到 `data/calib_result.txt` | numpy, OpenCV |
| [verify_handeye.py](verify_handeye.py) | **标定验收器(四类判据自动打分)**。对 `calib_data.csv` + `calib_result.txt` 做数据质量、多算法一致性、反投影闭环(金标准)、物理合理性四类判据,给总分并逐项给出 FAIL/PASS 与重采建议。**求解数据规模**用 `HAND_EYE_SOLVE_ROWS=N` 指定(见下) | numpy, OpenCV |
| [rgb_viewer.py](rgb_viewer.py) | **ArUco 检测显示 + TF 发布**。PyQt5 窗口实时显示相机画面;检测到 ID=12 时发布 `camera_color_optical_frame → camera_marker` 的动态 TF(实时位姿) | ROS2, OpenCV, **PyQt5** |
| [joint_angle_viewer.py](joint_angle_viewer.py) | ⚠️ **文件名与实际功能不符**。当前内容与 `rgb_viewer.py` 几乎相同——同样是 ArUco ID=12 检测 + 发布 `camera_color_optical_frame → camera_marker` TF。是 `rgb_viewer.py` 的旧版/前身,二选一运行即可 | ROS2, OpenCV, PyQt5 |

### 四个脚本的定位与选择

```
采集数据  calib_data_collector.py   → data/calib_data.csv
             │ (自动: 检测到 Tag 就采样 6 关节角 + tag 位姿)
             ▼
离线求解  calib_compute_handeye.py  → data/calib_result.txt
             │ (正运动学 + calibrateHandEye,解出 link6_T_camera)
             ▼
标定验收  verify_handeye.py          → 四类判据打分, 达标才能用
             │ (数据质量 / 多算法一致性 / 反投影闭环 / 物理合理性)
             ▼
实时发布  rgb_viewer.py             → 发布 camera_marker TF
             │ (或旧版 joint_angle_viewer.py)
             ▼
落地应用  static_transform_publisher (link6 → camera_link)
             参数来自 handeye_calib.yaml / calib_result.txt
```

`rgb_viewer.py`(当前使用)与 `joint_angle_viewer.py`(旧版)功能重叠,`rgb_viewer.py` 的注释
中残留旧值 ID=6,实际两个脚本都以 `TARGET_ID=12` 运行。

## 数据文件

```
data/
├── calib_data.csv       采集器输出(每行: index, timestamp, j1..j6, tag_tx..tz, tag_rx..rz)
└── calib_result.txt     求解器输出(link6_T_camera 齐次矩阵 + static TF 参数)
```

`data/calib_data.csv` 当前已有 171 组历史标定数据,即 `handeye_calib.yaml` 中所用标定的来源。

## 使用流程

```bash
# 前置: 先启动相机与机械臂
#   相机:  Realsense D435i (640x480x30, camera 命名空间)
#   机械臂: CAN can0, 并启动 agx_arm_ctrl 发布 /feedback/joint_states

# 1. 采集标定数据(多摆几个机械臂姿态,让 tag 持续可见,自动写入 CSV)
python3 calib_data_collector.py

# 2. 离线求解(可用 --list 查看 CSV)
python3 calib_compute_handeye.py

# 2.5 标定验收——达标才可投入使用
#   若 CSV 中混入了标定后新采集的数据, 用 HAND_EYE_SOLVE_ROWS 指定求解所用帧数
#   (例如上次标定用了 171 帧: HAND_EYE_SOLVE_ROWS=171 python3 verify_handeye.py)
python3 verify_handeye.py

# 3. 实时查看检测并发布 TF(替代已删除的 aruco_ros 节点)
python3 rgb_viewer.py
```

按 `Q` 键可退出各 GUI 窗口。采集与计算脚本的更多用法见文件顶部 docstring。

## 与 deleted piper_d435i_handeye 包的关系

`AR_bringup.launch.py` 中的 ArUco 检测节点与 EasyHandEye GUI 原由 `calibration/piper_d435i_handeye`
包提供,该包已删除(launch 中相关段已注释禁用)。本目录脚本即为替代:
标定数据采集与求解由上述脚本完成,实时 TF 由 `rgb_viewer.py` 发布,最终静态 TF
写入 `AR_bringup.launch.py` 的 `static_transform_publisher`(link6 → camera_link)。
