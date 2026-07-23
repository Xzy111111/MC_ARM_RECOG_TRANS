# driver

硬件驱动层：底盘、机械臂、激光雷达的驱动和模型描述。

---

## 包结构

```
driver/
├── car_sim/                  麦轮底盘 URDF 模型
├── vehicle_driver/           底盘串口驱动
├── mid360_driver/            Livox mid360 激光雷达驱动
└── piper_driver/             Piper 六轴机械臂驱动
    ├── agx_arm_ws/
    │   └── src/agx_arm_ros/src/
    │       ├── agx_arm_ctrl/          臂 CAN 控制器节点
    │       ├── agx_arm_description/   臂 URDF + robot_state_publisher
    │       ├── agx_arm_moveit/        MoveIt2 规划配置
    │       └── agx_arm_msgs/          自定义 ROS2 消息
    └── pyAgxArm/                      臂 CAN 协议 Python 库
```

---

## 1. car_sim — 麦轮底盘 URDF

麦轮底盘模型定义，提供 collision/visual/inertial 描述。

| 文件 | 说明 |
|------|------|
| `urdf/car_sim.urdf` | 纯底盘模型 |
| `urdf/car_with_piper.urdf.xacro` | 底盘 + Piper 臂组合模型 |
| `meshes/*.STL` | 底盘部件网格文件 |

**TF 结构：**
```
BODY（底盘主体）
├── F_B_R_Link（后右轮） ← continuous joint
├── F_B_L_Link（后左轮）
├── F_F_L_Link（前左轮）
├── F_F_R_Link（前右轮）
├── mid_Link（雷达） ← fixed joint mid360
└── base_link  ← 臂安装座（通过 body_to_base_link 连接）
```

---

## 2. vehicle_driver — 底盘串口驱动

**通信：** 串口 2Mbps，`/dev/ttyACM0`，24 字节自定义帧协议。

**输入：** `/cmd_vel`（geometry_msgs/Twist）
**输出：** 串口 24 字节帧 → STM32F407 → 麦轮电机

**协议格式：**
```
0xAA | vx_i16 | vy_i16 | vw_i16 | 使能 | 保留 | 反馈区(12B) | 0x55
```
速度编码：`count = clamp(round(value × 660), -660, 660)`，对应 ±1.0 m/s

**启动：**
```bash
ros2 run vehicle_driver vehicle_driver        # 启动驱动
ros2 run vehicle_driver keyboard_test          # 键盘遥控
```

**参数：** `port`, `baudrate`, `send_rate_hz`, `cmd_timeout_sec`, `max_vx/vy/wz`

---

## 3. mid360_driver — Livox mid360 雷达驱动

轻量级实现，不依赖 Livox-SDK2，直接 UDP 通信。

**特性：**
- 自动获取雷达 IP，无需手动配置
- 支持多雷达
- 支持点云坐标变换（旋转 + 平移）
- 支持距离筛选和轴向筛选
- 支持 IMU 低通滤波

**启动：**
```bash
ros2 launch mid360_driver mid360_driver.launch.py
```

---

## 4. piper_driver — Piper 六轴机械臂驱动

### 架构

```
agx_arm_ctrl_single_node（Python ROS2 节点）
    ↓
pyAgxArm（Python CAN 协议库）
    ↓
python-can / socketcan
    ↓
CAN bus can0 @ 1Mbps
    ↓
Piper 臂主控
```

### 组件说明

| 包 | 说明 |
|----|------|
| `agx_arm_ctrl` | 臂控制器 ROS2 节点 (`agx_arm_ctrl_single`) |
| `agx_arm_description` | 臂 URDF + display.launch.py（RSP + JSPG + rviz） |
| `agx_arm_moveit` | MoveIt2 运动规划配置 |
| `agx_arm_msgs` | 自定义消息（ArmStatus, GripperStatus, HandCmd 等） |
| `pyAgxArm` | 底层 CAN 协议实现，支持 Piper/Nero 系列固件版本 |

### 启动

```bash
# 仅启动控制器（AR_bringup.launch.py 已集成）
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py arm_type:=piper can_port:=can0

# 启动控制器 + 显示（含 rviz）
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py arm_type:=piper can_port:=can0
```

### CAN 协议

基于 CAN 2.0 标准帧，通过 0x711/0x712 隧道承载 CAN FD 载荷。
帧格式：`55 AA` 头 + 序列号 + 数据分片 + CRC16。
支持固件版本：S-V1.8-3~S-V1.8-9+。

### 控制接口

| 话题 | 类型 | 说明 |
|------|------|------|
| `control/joint_states` | JointState | 关节角度控制（JSPG 滑块输出） |
| `control/move_j` | JointState | 关节空间运动 |
| `control/move_p` | PoseStamped | 笛卡尔位置运动 |
| `control/move_l` | PoseStamped | 直线插补运动 |
| `control/move_c` | PoseArray | 圆弧插补运动 |
| `feedback/joint_states` | JointState | 关节角度反馈 |
| `feedback/tcp_pose` | PoseStamped | TCP 位姿反馈 |
| `feedback/arm_status` | AgxArmStatus | 臂状态反馈 |
| `enable_agx_arm` | SetBool service | 使能/失能 |
| `move_home` | Empty service | 回零 |
| `emergency_stop` | Empty service | 急停 |
