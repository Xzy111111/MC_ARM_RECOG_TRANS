# vehicle_driver - 底盘串口驱动

## 架构

```
/cmd_vel (geometry_msgs/Twist)
    |
    v
VehicleDriverNode (100Hz 定时器)
    |
    v
SerialPort (2Mbps, 8N1, /dev/ttyACM0)
    |
    v
STM32F407 -> 麦克纳麦轮电机
```

## 协议格式 (24字节帧)

| 偏移 | 字段 | 编码 | 说明 |
|------|------|------|------|
| 0 | frame_head | 0xAA | 帧头 |
| 1-2 | vx_rx | int16 big-endian | 线速度 X (+-660 = +-1.0 m/s) |
| 3-4 | vy_rx | int16 big-endian | 线速度 Y (+-660 = +-1.0 m/s) |
| 5-6 | vw_rx | int16 big-endian | 角速度 Z (+-660 = +-1.0 rad/s) |
| 7 | s_rx[0] | 0x01=enable, 0x02=lock | 使能标志 |
| 8 | s_rx[1] | 0x01=enable, 0x00=lock | 使能标志 |
| 9-10 | s_rx[2..3] | uint8 | 保留 |
| 11-22 | 反馈区 | - | 底盘反馈(上位机发送时置0) |
| 23 | frame_tail | 0x55 | 帧尾 |

### 编码公式

```
count = clamp(round(value * 660.0), -660, 660)
```

## 使用

### 启动驱动节点

```bash
ros2 run vehicle_driver vehicle_driver
```

### 启动键盘测试

```bash
ros2 run vehicle_driver keyboard_test
```

## 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| port | /dev/ttyACM0 | 串口设备路径 |
| baudrate | 2000000 | 串口波特率 |
| send_rate_hz | 100.0 | 指令发送频率 |
| cmd_timeout_sec | 0.5 | 指令超时(超时后自动停) |
| max_vx | 1.0 | X方向速度上限 (m/s) |
| max_vy | 1.0 | Y方向速度上限 (m/s) |
| max_wz | 1.0 | Z方向角速度上限 (rad/s) |
| debug_print_frame | False | 是否打印每帧hex |

## 键盘映射

| 按键 | 动作 | 说明 |
|------|------|------|
| i | 前进 | vx = +lin_max |
| , | 后退 | vx = -lin_max |
| j | 左移 | vy = +lin_max |
| l | 右移 | vy = -lin_max |
| u | 前左 | vx=+lin_max, vy=+lin_max |
| o | 前右 | vx=+lin_max, vy=-lin_max |
| m | 后左 | vx=-lin_max, vy=+lin_max |
| . | 后右 | vx=-lin_max, vy=-lin_max |
| a | 左转 | wz = +ang_max |
| d | 右转 | wz = -ang_max |
| k / Space | 停止 | vx=0, vy=0, wz=0 |
| w | 线速度+ | lin_max += 0.01 |
| x | 线速度- | lin_max -= 0.01 |
| e | 角速度+ | ang_max += 0.05 |
| c | 角速度- | ang_max -= 0.05 |
| q | 退出 | 关闭节点 |

初始线速度 0.10 m/s, 角速度 0.30 rad/s.

## 文件结构

```
vehicle_driver/
|-- package.xml
|-- setup.py
|-- setup.cfg
|-- vehicle_driver/
    |-- __init__.py
    |-- config.py              参数配置
    |-- vehicle_driver_node.py 主驱动节点(订阅/cmd_vel, 串口发送)
    |-- keyboard_test_node.py  键盘测试节点(发布/cmd_vel)
```
