# piper_control

Piper 机械臂 MoveIt 控制包（运行于 MC_ARM 移动底盘）。

规划坐标系 = `base_link`，与导航 TF 链 `map -> odom -> base_link` 无缝衔接（无 `world -> base_link`，避免 TF 多父节点冲突）。

## 包结构

```
piper_control/
├── launch/
│   └── piper_moveit.launch.py    # MoveIt 一站式启动（仿真 / 真机）
├── config/
│   ├── piper.srdf.xacro          # 语义描述: arm group(base_link→gripper_tcp), ready/home 姿态
│   ├── piper_arm.urdf.xacro      # URDF: base_link→link6→夹爪→gripper_tcp (含夹爪模型)
│   ├── kinematics.yaml           # KDL IK 求解器
│   ├── joint_limits.yaml         # 关节限位
│   ├── moveit_controllers.yaml   # arm_controller (joint1-6)
│   ├── ros2_controllers.yaml     # ros2_control mock 硬件配置
│   ├── sensors_3d.yaml           # octomap 传感器 (D435i 点云)
│   ├── initial_positions.yaml    # mock 初始关节角
│   └── moveit.rviz               # RViz 配置
└── urdf/
    └── piper_arm.urdf.xacro
```

## 核心概念

- **Planning group**: `arm` = `base_link -> gripper_tcp`（joint1~joint6）。`gripper_tcp` 是夹爪两指之间的抓取中心（固定 link，`gripper_base` 下 `[0,0,0.138]`），MoveIt 的 Interactive Marker 与 IK 目标点位于夹爪中心。
- **命名姿态**: `home` = 全 0；`ready` = `[0.0144, 0, 0, 0, 0, 1.0749]`（正常工作姿态）。
- **职责分离**: MoveIt 只管 joint1~joint6；夹爪由独立 `/control/gripper_cmd` 控制，不进 arm planning group。

## 启动机械臂 MoveIt

### 前置条件

1. 已构建并 source 工作空间：
   ```bash
   cd ~/MC_ARM_baseline_nav2_ok
   source install/setup.bash
   ```

2. 真机模式需 CAN 上电并激活接口：
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```
   确认接口状态：
   ```bash
   ip -s link show can0    # 应显示 state UP
   ```

### 方式一：纯仿真（无真机，测试规划/UI）

不需要机械臂与 CAN，用 mock 硬件回读：

```bash
source install/setup.bash
ros2 launch piper_control piper_moveit.launch.py \
    use_rviz:=true follow:=false use_hardware:=false
```

- `follow:=false`：MoveIt 订阅 `control/joint_states`（mock 回读）
- 可用于验证 planning group、`ready` 规划、RViz 显示（无需硬件）

### 方式二：真机（机械臂 + 夹爪）

```bash
sudo ip link set can0 up type can bitrate 1000000
source install/setup.bash
ros2 launch piper_control piper_moveit.launch.py \
    use_rviz:=true use_hardware:=true effector_type:=agx_gripper
```

关键参数：
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_hardware` | `false` | `true` = 启动真机驱动 agx_arm_ctrl（CAN） |
| `effector_type` | `none` | `agx_gripper` = 初始化 Piper 夹爪（必须，否则夹爪不可用） |
| `follow` | `true` | 订阅 `feedback/joint_states`（真机真实状态） |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_sensors` | `true` | 加载 octomap 传感器（需 D435i 相机）。无相机时设 `false` 避免 octomap 阻塞规划 |
| `arm_type` | `piper` | 臂型 |
| `speed_percent` | `100` | 机械臂运动速度百分比 |
| `tcp_offset` | `[0,0,0,0,0,0]` | 法兰参考点偏移 [x,y,z,r,p,y] |

> 注意：`use_sensors:=true`（默认）会订阅 `/camera/depth/color/points` 做 octomap。若 D435i 未启动且规划失败，设 `use_sensors:=false`。

### 启动后确认

```bash
# 1. planning group 与 ready 已加载
ros2 param get /move_group robot_description_semantic

# 2. 真机关节反馈
ros2 topic echo /feedback/joint_states

# 3. TF 链 (base_link→link1→...→link6→夹爪→gripper_tcp)
ros2 run tf2_ros tf2_echo base_link gripper_tcp

# 4. 机械臂执行 ready 姿态 (MoveIt 命名目标)
#    在 RViz Motion Planning 面板选 group=arm, 目标=ready, Plan & Execute
#    或编程调用:
#    move_group.setNamedTarget("ready"); move_group.plan(); move_group.execute();
```

## 夹爪控制

夹爪使用**独立命令话题** `/control/gripper_cmd`（不经过 MoveIt，也不被 `/control/joint_states` 状态值干扰）。

**消息约定**: `sensor_msgs/msg/JointState`，`name=["gripper"]`，`position[0]`=宽度（米），`effort[0]`=夹持力（牛）。

```bash
# 张开到 2cm，力 1N（单次发布即生效，无需高频）
ros2 topic pub --once /control/gripper_cmd sensor_msgs/msg/JointState \
    "{header: {stamp: now}, name: ['gripper'], position: [0.02], effort: [1.0]}"

# 完全闭合
ros2 topic pub --once /control/gripper_cmd sensor_msgs/msg/JointState \
    "{header: {stamp: now}, name: ['gripper'], position: [0.0], effort: [1.0]}"
```

夹爪状态反馈：`/feedback/gripper_status`（`agx_arm_msgs/msg/GripperStatus`）

```bash
ros2 topic echo /feedback/gripper_status
# width: 夹爪开度 (米); force: 夹持力 (牛); driver_enable_status: 驱动使能
```

> 参数范围：width 0~0.1m，force 0.5~3.0N（驱动自动 clamp）。

## 常见问题

**1. 夹爪不动作？**
- 确认启动时带 `effector_type:=agx_gripper`
- 确认 `can0` 为 UP 状态
- 夹爪用 `/control/gripper_cmd` 控制，不要发 `/control/joint_states`（该话题是状态反馈，不再控制夹爪）

**2. MoveIt 规划失败 / octomap 阻塞？**
- 未启动 D435i 时设 `use_sensors:=false`

**3. 真机机械臂不动？**
- 确认 `can0` 上电、`use_hardware:=true`
- 检查 `/feedback/joint_states` 是否有真实关节值

## 后续自主抓取流程（规划）

```
MoveIt pre_grasp → MoveIt grasp_pose → /control/gripper_cmd close
→ 检查 /feedback/gripper_status → MoveIt lift
```

- MoveIt `arm` 末端参考 = `gripper_tcp`（夹爪抓取中心）
- 目标 `geometry_msgs/PoseStamped` 交给 `MoveGroupInterface.plan()/execute()`，不依赖 RViz 手动拖动
