# exts/script — 手眼标定 & ArUco 工具脚本

本目录存放**脱离 ROS 包管理、直接以 `python3` 运行**的标定工具脚本。它们是手眼标定
`calibration/piper_d435i_handeye` 包被删除后的替代方案,完整覆盖从**数据采集 → 求解 →
验收 → TF 发布 → 落地**的整条链路。

> 标定对象: 相机（Intel RealSense D435i）固定在 Piper 机械臂第 6 轴（link6）上,即
> **Eye-on-Hand** 标定。标定板为 ArUco **ID=12**,边长 **0.128 m**。
>
> 坐标系约定: 标定使用的 tag 位姿来自 `cv2.solvePnP`（OpenCV 相机系, **+Z=光轴向前**,
> 即 `camera_color_optical_frame` 系）。因此求解出的 `link6_T_camera` 矩阵是 **optical 系**
> 的手眼变换,落地为 `link6 → camera_link` 时需做旋转补偿(见下文"落地"一节)。

## 文件说明

| 文件 | 作用 | 依赖 |
|------|------|------|
| [calib_data_collector.py](calib_data_collector.py) | **手眼标定数据采集器(全自动)**。实时显示相机画面并检测 ArUco;检测到目标 ID=12 时,自动读取 `/feedback/joint_states` 的 6 关节角度,连同 tag 位姿一起写入 `data/calib_data.csv`。**自动采集**策略:检测到 Tag + 0.5s 冷却才记录一帧(关节有变化即可),去重防连拍 | ROS2, OpenCV, **PySide2** |
| [calib_compute_handeye.py](calib_compute_handeye.py) | **手眼标定求解器**。读取 `calib_data.csv`,用内置 Piper 六轴正运动学(从 `car_with_piper.urdf.xacro` 提取)求 `base_T_link6`,配合 tag 位姿经 `cv2.calibrateHandEye`(Tsai-Lenz 为主,5 种算法对比)解出 `link6_T_camera`(optical 系)。输出 4×4 齐次矩阵 + static TF 命令行参数,保存到 `data/calib_result.txt`。支持 `--list` 查看 CSV | numpy, OpenCV |
| [verify_handeye.py](verify_handeye.py) | **标定验收器(四类判据自动打分)**。对 `calib_data.csv` + `calib_result.txt` 做:①数据质量、②多算法一致性、③反投影闭环(金标准)、④物理合理性,给总分并逐项 FAIL/PASS 与重采建议。**达标(总分≥阈值)才可投入使用**。若 CSV 混入标定后新数据,用 `HAND_EYE_SOLVE_ROWS=N` 指定参与求解的帧数 | numpy, OpenCV |
| [rgb_viewer.py](rgb_viewer.py) | **ArUco 检测显示 + TF 发布**。PyQt5 窗口实时显示相机画面;检测到 ID=12 时发布 `camera_color_optical_frame → camera_marker` 的动态 TF(实时位姿)。**当前实际使用的实时查看工具** | ROS2, OpenCV, **PyQt5** |
| [joint_angle_viewer.py](joint_angle_viewer.py) | ⚠️ **文件名与实际功能不符**。当前内容与 `rgb_viewer.py` 几乎相同——同样是 ArUco ID=12 检测 + 发布 `camera_color_optical_frame → camera_marker` TF。是 `rgb_viewer.py` 的旧版/前身,二选一运行即可 | ROS2, OpenCV, PyQt5 |

### 五个脚本的定位与链路

```
采集数据  calib_data_collector.py   → data/calib_data.csv
             │ (自动: 检测到 Tag 就采样 6 关节角 + tag 位姿)
             ▼
离线求解  calib_compute_handeye.py → data/calib_result.txt
             │ (正运动学 + calibrateHandEye, 解出 link6_T_camera, optical 系)
             ▼
标定验收  verify_handeye.py         → 四类判据打分, 达标才可用
             │ (数据质量 / 多算法一致性 / 反投影闭环 / 物理合理性)
             ▼
实时验证  rgb_viewer.py             → 发布 camera_marker TF 实测
             │ (或旧版 joint_angle_viewer.py)
             ▼
落地应用  static_transform_publisher (link6 → camera_link)
             参数来自 calib_result.txt, 需做旋转补偿(见下)
```

`rgb_viewer.py`(当前使用)与 `joint_angle_viewer.py`(旧版)功能重叠,二选一运行即可。

## 数据文件

```
data/
├── calib_data.csv       采集器输出(每行: index, timestamp, j1..j6, tag_tx..tz, tag_rx..rz)
├── calib_result.txt     求解器输出(link6_T_camera 齐次矩阵 + static TF 参数)
└── calib_data.csv.bak_* 重采前的旧数据备份(每次重采前 `cp` 一份)
```

`data/calib_data.csv` 当前为 **234 组**(2026-08-11 重采),`calib_result.txt` 为对应求解结果。

## 使用流程

```bash
# 前置: 先启动相机与机械臂
#   相机:  Realsense D435i (640x480x30, camera 命名空间)
#   机械臂: CAN can0, 并启动 agx_arm_ctrl 发布 /feedback/joint_states
#   完整环境: ros2 launch bringup AR_bringup.launch.py

# 0. 重采前备份旧数据(保留表头)
cp data/calib_data.csv data/calib_data.csv.bak_$(date +%Y%m%d)
head -1 data/calib_data.csv > /tmp/hdr && mv /tmp/hdr data/calib_data.csv

# 1. 采集标定数据(多摆机械臂姿态, 让 tag 持续可见, 自动写入 CSV)
python3 calib_data_collector.py

# 2. 离线求解(可用 --list 查看 CSV)
python3 calib_compute_handeye.py

# 2.5 标定验收——达标才可投入使用
#   若 CSV 中混入了标定后新采集的数据, 用 HAND_EYE_SOLVE_ROWS 指定求解所用帧数
python3 verify_handeye.py

# 3. 实时查看检测并发布 TF(替代已删除的 aruco_ros 节点)
python3 rgb_viewer.py
```

按 `Q` 键可退出各 GUI 窗口。采集与计算脚本的更多用法见文件顶部 docstring。

## 落地: link6 → camera_link(⚠️ 关键)

求解器输出的矩阵是 **optical 系**(+Z=光轴)。但 Realsense 相机里 `camera_link` 的光轴是
**+X**(相机驱动发布 `camera_link→camera_color_frame→camera_color_optical_frame` 内部链,
把 +X 转到 +Z)。**直接把求解器的 rpy 填进 `static_transform_publisher --frame-id link6
--child-frame-id camera_link` 会导致光轴偏 90°。**

正确做法(旋转补偿):
```
link6_T_camera_link = link6_T_optical × inv(camera_link→optical)
```
其中 `camera_link→optical` 是相机进程发布的固定变换(平移 [0, 0.015, 0] 附近, 旋转约
绕 Y 90°)。补偿后填进 `AR_bringup.launch.py` 的 `static_transform_publisher`。

落地后可用 `ros2 run tf2_ros tf2_echo world camera_color_optical_frame` 验证:光轴(+Z)
应指向机械臂前方(与 link6 末端方向一致)。

## 采集质量要点(影响验收成败)

`verify_handeye.py` 四类判据中, **反投影闭环(金标准)** 和 **多算法一致性** 最容易被数据
质量拖垮。重采时务必:

1. **tag 距离 0.3~0.8m 且远近都要有**(不要一直待在 ~1m, 深度方向变化是平移约束来源)
2. **大幅摆动所有关节**, 尤其 J1(腰, ±60°) / J2(大臂) / J3(小臂), 每次变 15°+
3. **J6(末端) 转到不同角度**(0°/60°/120°/180°), 提供旋转约束
4. **每个姿态只采 1~2 条就换**, 避免同一姿态重复
5. 采集窗口底部内参显示 **OK**(非 default), 否则 tag 位姿测量不准

## 与 deleted piper_d435i_handeye 包的关系

`AR_bringup.launch.py` 中的 ArUco 检测节点与 EasyHandEye GUI 原由 `calibration/piper_d435i_handeye`
包提供,该包已删除(launch 中相关段已注释禁用)。本目录脚本即为替代:
标定数据采集与求解由上述脚本完成,实时 TF 由 `rgb_viewer.py` 发布,最终静态 TF
写入 `AR_bringup.launch.py` 的 `static_transform_publisher`(link6 → camera_link)。
