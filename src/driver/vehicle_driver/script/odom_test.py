#!/usr/bin/env python3
"""
订阅 /odin1/odometry_highfreq 并实时可视化 Odometry 原生数据。

显示内容（纯原生，不做任何滤波/解包处理）：
1) position  x/y/z
2) orientation → roll/pitch/yaw + raw quaternion x/y/z/w
3) linear  velocity  x/y/z
4) angular velocity  x/y/z
5) covariance 对角元（pose 定位置信度）

============================================================================
深度分析遗漏点（代码目前未覆盖，但对接下来的优化至关重要）：
  A) x/y 平面轨迹图（top-down view）— 缺少空间感，只看时间序列看不出漂移方向
  B) twist 积分位置 vs pose 直接位置的一致性对比 → 检测里程计内冲突
  C) 协方差椭圆可视化 → covariance 矩阵包含完整的相关性信息，对角元只是冰山一角
  D) 数据落盘 + bag 回放模式 → 当前只能实时看，无法对比不同算法版本
  E) 发布频率稳定性统计 → 高频 odom 掉帧会直接影响控制稳定性
  F) 数值稳定性检测 → NaN / Inf / 零值卡死 的实时告警
  G) 多坐标系验证 → frame_id/child_frame_id 是否与 TF 树一致

运行方式：
    python3 /home/inkc/inkc/Rc2026/src/exts/script/odom_test.py
============================================================================
"""

import math
import signal
import threading
import time                     # 用于接收端延迟统计（monotonic 时钟）
from collections import deque   # O(1) 两端操作，适合滚动窗口

import matplotlib.pyplot as plt
import numpy as np              # 协方差对角提取、统计分析
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ============================================================================
# 可调参数
# ============================================================================
ODOM_TOPIC = '/Odometry'
NODE_NAME = 'odom_realtime_visualizer'

PLOT_WINDOW_SEC = 6.0          # 滚动窗口时长（秒），过长会拖慢绘图
PLOT_REFRESH_HZ = 20.0          # 图表刷新率（帧/秒），与 ROS spin 频率无关

WINDOW_TITLE = 'Realtime Odometry Viewer'
FIG_SIZE = (12, 14)             # 加宽留出图例空间
GRID_ALPHA = 0.28
LINE_WIDTH = 1.3

COLORS = {
    'x': '#d62728',      # 红色 — x 轴
    'y': '#2ca02c',      # 绿色 — y 轴
    'z': '#1f77b4',      # 蓝色 — z 轴
    'w': '#9467bd',      # 紫色 — 四元数 w
}

# ============================================================================
# 四元数 → 欧拉角 转换（与 tf_transformations 一致，不依赖额外包）
# ============================================================================
def quaternion_to_rpy(x, y, z, w):
    """
    四元数 → roll/pitch/yaw（ZYX 欧拉角）。
    注意：当 pitch 接近 ±90° 时存在万向锁(gimbal lock)，此时 yaw/roll 退化。
    对于地面机器人（pitch ≈ 0），此转换完全安全。
    """
    # roll  (绕 x 轴)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch（绕 y 轴）— 注意 atan2 → asin 的区别，asin 在 ±1 处饱和
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # 万向锁保护
    else:
        pitch = math.asin(sinp)

    # yaw  （绕 z 轴）— 地面机器人最重要的角度
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ============================================================================
# Odometry 订阅节点
# ============================================================================
class OdometryVisualizer(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.stop_event = threading.Event()
        self._lock = threading.Lock()

        # ---- 时间轴 ----
        self.t_ros = deque()        # ROS header.stamp（传感器数据时刻）
        self.t_recv = deque()       # 接收时刻（time.monotonic），用于延迟检测

        # ---- pose position ----
        self.px = deque()
        self.py = deque()
        self.pz = deque()

        # ---- orientation rpy（从四元数转换而来，仅用于可读性） ----
        self.roll = deque()
        self.pitch = deque()
        self.yaw = deque()

        # ---- raw quaternion（原生朝向数据，不做任何处理） ----
        self.qx = deque()
        self.qy = deque()
        self.qz = deque()
        self.qw = deque()

        # ---- linear velocity ----
        self.vx = deque()
        self.vy = deque()
        self.vz = deque()

        # ---- angular velocity ----
        self.wx = deque()
        self.wy = deque()
        self.wz = deque()

        # ---- 协方差对角元（定位置信度的量化指标） ----
        #   pose covariance 6x6 对角顺序: x, y, z, roll, pitch, yaw
        self.cov_xx = deque()       # x 方向位置方差
        self.cov_yy = deque()       # y 方向位置方差
        self.cov_zz = deque()       # z 方向位置方差
        self.cov_aa = deque()       # yaw 角度方差（绕 z 轴）

        self._last_recv = None      # 上一条消息的接收时刻

        # 创建订阅（QoS 默认是 ReliabilityPolicy.SYSTEM_DEFAULT，
        # 高频 odom 建议使用 BEST_EFFORT + KEEP_LAST）
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_callback, QoSProfile(depth=5000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f'Subscribing: {ODOM_TOPIC}')

    # ------------------------------------------------------------------
    # 滚动窗口裁剪：只保留最近 PLOT_WINDOW_SEC 秒的数据
    # 避免 deque 无限增长消耗内存
    # ------------------------------------------------------------------
    def _trim_old(self, now_ts):
        min_ts = now_ts - PLOT_WINDOW_SEC
        while self.t_ros and self.t_ros[0] < min_ts:
            self.t_ros.popleft()
            self.t_recv.popleft()
            self.px.popleft()
            self.py.popleft()
            self.pz.popleft()
            self.roll.popleft()
            self.pitch.popleft()
            self.yaw.popleft()
            self.qx.popleft()
            self.qy.popleft()
            self.qz.popleft()
            self.qw.popleft()
            self.vx.popleft()
            self.vy.popleft()
            self.vz.popleft()
            self.wx.popleft()
            self.wy.popleft()
            self.wz.popleft()
            self.cov_xx.popleft()
            self.cov_yy.popleft()
            self.cov_zz.popleft()
            self.cov_aa.popleft()

    # ------------------------------------------------------------------
    # Odometry 回调函数
    #   - 只做「读 + 存」，不做任何滤波/平滑/解包
    #   - 耗时操作（如 quaternion_to_rpy）在此同步执行，高频下应当评估开销
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        now_recv = time.monotonic()                     # 接收端本地时间
        now_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # 传感器时间

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        pcov = msg.pose.covariance         # 长度为 36 的数组（6x6 行优先）
        # tcov = msg.twist.covariance      # twist 协方差（暂未使用）

        # 四元数 → RPY（仅为了可读性，不参与任何控制逻辑）
        roll, pitch, yaw = quaternion_to_rpy(
            float(o.x), float(o.y), float(o.z), float(o.w),
        )

        with self._lock:
            self.t_ros.append(now_ts)
            self.t_recv.append(now_recv)

            self.px.append(float(p.x))
            self.py.append(float(p.y))
            self.pz.append(float(p.z))

            self.roll.append(roll)
            self.pitch.append(pitch)
            self.yaw.append(yaw)

            self.qx.append(float(o.x))
            self.qy.append(float(o.y))
            self.qz.append(float(o.z))
            self.qw.append(float(o.w))

            self.vx.append(float(t.linear.x))
            self.vy.append(float(t.linear.y))
            self.vz.append(float(t.linear.z))

            self.wx.append(float(t.angular.x))
            self.wy.append(float(t.angular.y))
            self.wz.append(float(t.angular.z))

            # 提取协方差对角元（6x6 矩阵的对角索引: 0,7,14,21,28,35）
            # 当前只关心: x, y, z 位置方差 + yaw 角度方差
            self.cov_xx.append(float(pcov[0])  if len(pcov) > 0  else 0.0)
            self.cov_yy.append(float(pcov[7])  if len(pcov) > 7  else 0.0)
            self.cov_zz.append(float(pcov[14]) if len(pcov) > 14 else 0.0)
            self.cov_aa.append(float(pcov[35]) if len(pcov) > 35 else 0.0)

            # 接收端本地时间记录（为后续延迟分析保留，暂未绘图）
            self._last_recv = now_recv

            self._trim_old(now_ts)

    # ------------------------------------------------------------------
    # 快照：线程安全地导出当前所有数据（深拷贝 list）
    # 在主线程（matplotlib）中调用，避免直接在绘图线程中加锁
    # ------------------------------------------------------------------
    def snapshot(self):
        with self._lock:
            return {
                't_ros': list(self.t_ros),
                't_recv': list(self.t_recv),
                'px': list(self.px),
                'py': list(self.py),
                'pz': list(self.pz),
                'roll': list(self.roll),
                'pitch': list(self.pitch),
                'yaw': list(self.yaw),
                'qx': list(self.qx),
                'qy': list(self.qy),
                'qz': list(self.qz),
                'qw': list(self.qw),
                'vx': list(self.vx),
                'vy': list(self.vy),
                'vz': list(self.vz),
                'wx': list(self.wx),
                'wy': list(self.wy),
                'wz': list(self.wz),
                'cov_xx': list(self.cov_xx),
                'cov_yy': list(self.cov_yy),
                'cov_zz': list(self.cov_zz),
                'cov_aa': list(self.cov_aa),
            }

    # ------------------------------------------------------------------
    # 数值健康检查：在当前数据中扫描 NaN / Inf / 异常零值
    # 返回 (has_problem, report_string)
    #   【遗漏点 F】当前未被主循环调用，需要手动集成
    # ------------------------------------------------------------------
    def check_numerical_health(self, snap) -> tuple:
        problems = []
        checks = [
            ('px', snap['px']), ('py', snap['py']), ('pz', snap['pz']),
            ('vx', snap['vx']), ('vy', snap['vy']), ('vz', snap['vz']),
        ]
        for name, series in checks:
            if not series:
                continue
            arr = np.array(series, dtype=np.float64)
            nan_cnt = np.sum(np.isnan(arr))
            inf_cnt = np.sum(np.isinf(arr))
            if nan_cnt > 0:
                problems.append(f'{name}: {nan_cnt} NaN')
            if inf_cnt > 0:
                problems.append(f'{name}: {inf_cnt} Inf')
            # 检测连续零值卡死（最后 50 个点全部为零）
            if len(arr) >= 50 and np.all(arr[-50:] == 0.0):
                problems.append(f'{name}: 卡死（最近50点全零）')
        return (len(problems) > 0, '; '.join(problems) if problems else 'OK')


# ============================================================================
# 辅助：自适应 y 轴范围
#   动态调整 ylim 让曲线始终居中，min_span 防止静止时过度放大噪声
# ============================================================================
def set_axis_ylim(ax, series_list, min_span):
    values = [v for series in series_list for v in series]
    if not values:
        return
    y_min = min(values)
    y_max = max(values)
    span = max(min_span, y_max - y_min)
    margin = 0.18 * span
    ax.set_ylim(y_min - margin, y_max + margin)


# ============================================================================
# 绘图主循环
# ============================================================================
def run_plot_window(node: OdometryVisualizer):
    """
    6 行子图布局：
    0: position  x/y/z                      [m]
    1: orientation  roll/pitch/yaw (raw)    [rad]
    2: linear velocity  x/y/z               [m/s]
    3: angular velocity  x/y/z              [rad/s]
    4: raw quaternion  x/y/z/w              [1]
    5: covariance 对角元 (xx/yy/zz + yaw)    [m², rad²]
    """
    fig, axes = plt.subplots(6, 1, figsize=FIG_SIZE, sharex=True)
    fig.canvas.manager.set_window_title(WINDOW_TITLE)
    fig.suptitle(f'/{ODOM_TOPIC.lstrip("/")}', fontsize=13, fontweight='bold')
    # ------------------------------------------------------------------
    # 子图 0: position
    # 用途：机器人在空间中的位置轨迹，漂移检测
    # 注意：当机器人静止时，这些值应基本不变；若有周期性波动说明 IMU 融合异常
    # 最佳呈现：从零开始，长时间尺度下观察是否回中 / 发散
    # 改进点 A：这里只是时间序列，缺少 xy 平面轨迹图(top-down view)
    # ------------------------------------------------------------------
    px_line, = axes[0].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='pos x')
    py_line, = axes[0].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='pos y')
    pz_line, = axes[0].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='pos z')
    axes[0].set_ylabel('position (m)')
    axes[0].set_title('Position', loc='left', fontsize=10)
    axes[0].grid(True, alpha=GRID_ALPHA)
    axes[0].legend(loc='upper left', ncol=3, fontsize=8)

    # ------------------------------------------------------------------
    # 子图 1: orientation RPY
    # 用途：检查姿态估计是否正确，yaw 是否有跳变 / 漂移
    # 注意：yaw 在 ±π 跳变是反三角函数的正常行为，不要当成异常
    #       roll/pitch 应当是零均值的（地面机器人假设），若出现常值偏置
    #       说明 IMU 初始化未完成或加速度计偏置未补偿
    # 改进点 B：与 twist 角速度积分对比可检测估计器内部一致性
    # ------------------------------------------------------------------
    roll_line, = axes[1].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='roll')
    pitch_line, = axes[1].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='pitch')
    yaw_line, = axes[1].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='yaw (raw ±π)')
    axes[1].set_ylabel('orientation (rad)')
    axes[1].set_title('RPY (raw, no unwrap)', loc='left', fontsize=10)
    axes[1].grid(True, alpha=GRID_ALPHA)
    axes[1].legend(loc='upper left', ncol=3, fontsize=8)

    # ------------------------------------------------------------------
    # 子图 2: linear velocity
    # 用途：底盘实际移动速度的反馈值
    # 注意：x 为前进方向（机器人坐标系），y 为横向（侧滑量）
    #       z 在平整路面应接近零，非零意味着车体颠簸或安装面不平
    # 改进点 B：速度积分 Δp = ∫ v·dt 应与 pose 变化一致（一致性检验）
    # ------------------------------------------------------------------
    vx_line, = axes[2].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='linear x')
    vy_line, = axes[2].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='linear y')
    vz_line, = axes[2].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='linear z')
    axes[2].set_ylabel('linear (m/s)')
    axes[2].set_title('Linear Velocity', loc='left', fontsize=10)
    axes[2].grid(True, alpha=GRID_ALPHA)
    axes[2].legend(loc='upper left', ncol=3, fontsize=8)

    # ------------------------------------------------------------------
    # 子图 3: angular velocity
    # 用途：旋转速度，z 轴对应车体旋转（偏航角速度）
    # 注意：wz 控制转弯；wx/wy 在平整路面应接近零
    # ------------------------------------------------------------------
    wx_line, = axes[3].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='angular x')
    wy_line, = axes[3].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='angular y')
    wz_line, = axes[3].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='angular z')
    axes[3].set_ylabel('angular (rad/s)')
    axes[3].set_title('Angular Velocity', loc='left', fontsize=10)
    axes[3].grid(True, alpha=GRID_ALPHA)
    axes[3].legend(loc='upper left', ncol=3, fontsize=8)

    # ------------------------------------------------------------------
    # 子图 4: raw quaternion
    # 用途：最原生的朝向数据，滤波前的第一手信息
    # 注意：四元数必须满足单位范数约束 qx²+qy²+qz²+qw² ≈ 1
    #       若偏离 1 说明上游里程计融合出现数值误差
    #       此外 qw 接近 1 且 qx,qy,qz 接近 0 表示朝向接近零姿态
    # ------------------------------------------------------------------
    qx_line, = axes[4].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='quat x')
    qy_line, = axes[4].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='quat y')
    qz_line, = axes[4].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='quat z')
    qw_line, = axes[4].plot([], [], color=COLORS['w'], linewidth=LINE_WIDTH, label='quat w')
    axes[4].set_ylabel('quaternion')
    axes[4].set_title('Raw Quaternion', loc='left', fontsize=10)
    axes[4].grid(True, alpha=GRID_ALPHA)
    axes[4].legend(loc='upper left', ncol=4, fontsize=8)

    # ------------------------------------------------------------------
    # 子图 5: covariance 对角元（定位置信度）
    # 用途：判断里程计发散程度
    #   cov_xx/cov_yy → 位置不确定性（越大越不可信）
    #   cov_aa       → 角度不确定性（yaw 方差）
    # 注意：
    #   - 协方差突然跳增通常意味着：打滑 / 碰撞 / IMU 饱和 / 视觉追踪丢失
    #   - 协方差持续增长（不发散）说明 EKF 收敛但融合权重异常
    #   - 改进点 C：完整协方差椭圆 + 相关系数比对角元更有价值
    # ------------------------------------------------------------------
    cov_xx_line, = axes[5].plot([], [], color=COLORS['x'], linewidth=LINE_WIDTH, label='cov xx')
    cov_yy_line, = axes[5].plot([], [], color=COLORS['y'], linewidth=LINE_WIDTH, label='cov yy')
    cov_zz_line, = axes[5].plot([], [], color=COLORS['z'], linewidth=LINE_WIDTH, label='cov zz')
    cov_aa_line, = axes[5].plot([], [], color=COLORS['w'], linewidth=LINE_WIDTH, label='cov aa (yaw)')
    axes[5].set_ylabel('covariance')
    axes[5].set_title('Pose Covariance (diag)', loc='left', fontsize=10)
    axes[5].grid(True, alpha=GRID_ALPHA)
    axes[5].legend(loc='upper left', ncol=4, fontsize=8)

    # 最后一行（原序号 6）时间健康度已删除，共 6 张子图

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    plt.show(block=False)

    # ------------------------------------------------------------------
    # 窗口关闭事件 → 停止节点
    # ------------------------------------------------------------------
    def _on_close(_event):
        node.stop_event.set()
    fig.canvas.mpl_connect('close_event', _on_close)

    sleep_dt = 1.0 / max(1.0, PLOT_REFRESH_HZ)

    # ------------------------------------------------------------------
    # 主绘循环
    #   - 每次迭代从 node 取快照，更新所有 6 张子图
    #   - 使用 plt.pause(sleep_dt) 让 GUI 事件循环有机会刷新
    #   - 性能瓶颈：数据量增大后 set_data + set_ylim 可能卡顿
    #     优化方向：降低 PLOT_REFRESH_HZ 或减少 PLOT_WINDOW_SEC
    # ------------------------------------------------------------------
    try:
        while (not node.stop_event.is_set()) and plt.get_fignums():
            snap = node.snapshot()
            if snap['t_ros']:
                # 使用 ROS 时间戳为基准，转成相对时间（避免大数＋小数精度问题）
                t0 = snap['t_ros'][0]
                t_rel = [ts - t0 for ts in snap['t_ros']]

                # 更新 position
                px_line.set_data(t_rel, snap['px'])
                py_line.set_data(t_rel, snap['py'])
                pz_line.set_data(t_rel, snap['pz'])

                # 更新 orientation RPY
                roll_line.set_data(t_rel, snap['roll'])
                pitch_line.set_data(t_rel, snap['pitch'])
                yaw_line.set_data(t_rel, snap['yaw'])

                # 更新 linear velocity
                vx_line.set_data(t_rel, snap['vx'])
                vy_line.set_data(t_rel, snap['vy'])
                vz_line.set_data(t_rel, snap['vz'])

                # 更新 angular velocity
                wx_line.set_data(t_rel, snap['wx'])
                wy_line.set_data(t_rel, snap['wy'])
                wz_line.set_data(t_rel, snap['wz'])

                # 更新 raw quaternion
                qx_line.set_data(t_rel, snap['qx'])
                qy_line.set_data(t_rel, snap['qy'])
                qz_line.set_data(t_rel, snap['qz'])
                qw_line.set_data(t_rel, snap['qw'])

                # 更新 covariance
                cov_xx_line.set_data(t_rel, snap['cov_xx'])
                cov_yy_line.set_data(t_rel, snap['cov_yy'])
                cov_zz_line.set_data(t_rel, snap['cov_zz'])
                cov_aa_line.set_data(t_rel, snap['cov_aa'])

                # 第 6 张子图（时间健康度）已删除，无 delay/dt 更新

                # x 轴范围（滚动窗口）
                x_max = max(PLOT_WINDOW_SEC, t_rel[-1] if t_rel else PLOT_WINDOW_SEC)
                x_min = max(0.0, x_max - PLOT_WINDOW_SEC)
                axes[5].set_xlim(x_min, x_max)

                # 自适应 y 轴（每个子图独立）
                set_axis_ylim(axes[0], [snap['px'], snap['py'], snap['pz']], min_span=0.01)
                set_axis_ylim(axes[1], [snap['roll'], snap['pitch'], snap['yaw']], min_span=0.1)
                set_axis_ylim(axes[2], [snap['vx'], snap['vy'], snap['vz']], min_span=0.05)
                set_axis_ylim(axes[3], [snap['wx'], snap['wy'], snap['wz']], min_span=0.05)
                set_axis_ylim(axes[4], [snap['qx'], snap['qy'], snap['qz'], snap['qw']], min_span=0.01)
                set_axis_ylim(axes[5], [snap['cov_xx'], snap['cov_yy'], snap['cov_zz'], snap['cov_aa']], min_span=1e-6)

            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                pass
            time.sleep(sleep_dt)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, closing...')
    finally:
        node.stop_event.set()
        plt.close('all')


# ============================================================================
# 主函数
# ============================================================================
def main():
    rclpy.init()
    node = OdometryVisualizer()

    # MultiThreadedExecutor 让 spin 与 callback 在不同的线程运行
    # 防止 callback 中的计算阻塞 spin 的 I/O
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def _spin_worker():
        try:
            executor.spin()
        except Exception:
            pass

    spin_thread = threading.Thread(target=_spin_worker, daemon=True, name='odom_spin_thread')
    spin_thread.start()

    # 保存旧的 SIGINT handler，退出时恢复
    prev_sigint_handler = signal.getsignal(signal.SIGINT)

    def _sigint_handler(_signum, _frame):
        if not node.stop_event.is_set():
            node.get_logger().info('SIGINT, shutting down...')
        node.stop_event.set()
        plt.close('all')

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        run_plot_window(node)
    finally:
        node.stop_event.set()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        signal.signal(signal.SIGINT, prev_sigint_handler)


if __name__ == '__main__':
    main()
