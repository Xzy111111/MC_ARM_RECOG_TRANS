#!/usr/bin/env python3
"""
基于 /odin1/odometry_highfreq 的旋转中心标定 + 实时可视化。

功能融合：
  ─ calibrate_car_center.py : 圆拟合标定 odin1_base_link → base_link 外参
  ─ odom_test.py            : 实时绘制 Odometry 全部数据 + XY 轨迹

录制并可视化:
  - XY 轨迹（直观判断是否为圆） + 位置 / 姿态 / 速度
  - 后台按时间均匀重采样（20Hz），Ctrl+C 自动跑圆拟合 → 输出标定结果
"""

import math
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ==================== 配置 ====================

ODOM_TOPIC = '/Odometry'

# ---- 校准参数 ----
RESAMPLE_INTERVAL = 0.05       # 校准采样间隔（秒），0.05=20Hz
MIN_POINTS = 20                # 最少有效拟合点数
PRINT_DECIMALS = 6             # 打印精度
USE_PITCH_ROLL_COMP = True     # roll/pitch 补偿开关
OUTLIER_MAD_THRESHOLD = 3.0    # MAD 剔除阈值（σ 倍数）

# ---- 可视化参数 ----
PLOT_WINDOW_SEC = 30.0         # 时间窗（秒），校准要转多圈，给长一点
PLOT_REFRESH_HZ = 15.0         # 绘图刷新率
WINDOW_TITLE = 'Calibration — XY + Odometry'
FIG_SIZE = (10, 13)            # 5 行子图需要更大
GRID_ALPHA = 0.28
LINE_WIDTH = 1.3
COLORS = {'x': '#d62728', 'y': '#2ca02c', 'z': '#1f77b4'}

# ---- 终端状态输出 ----
STATUS_INTERVAL = 2.0


# ==================== 数据结构 ====================

@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp: float


# ==================== 数学工具 ====================

def quat_normalize(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (0.0, 0.0, 0.0, 1.0) if n <= 0 else (qx / n, qy / n, qz / n, qw / n)


def quat_to_rpy(qx, qy, qz, qw):
    qx, qy, qz, qw = quat_normalize(qx, qy, qz, qw)
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def quat_to_rot_matrix(qx, qy, qz, qw):
    """R: v_odom = R · v_odin1"""
    qx, qy, qz, qw = quat_normalize(qx, qy, qz, qw)
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def mat_transpose_3x3(m):
    return [[m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]]]


def mat_vec_mul_3x3(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def solve_3x3(A, b):
    """高斯-约当消元解 3×3 方程组 A·x = b。"""
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(A[r][col]))
        if abs(A[pivot][col]) < 1e-12:
            raise RuntimeError(f"solve_3x3: singular matrix at col {col}")
        A[col], A[pivot] = A[pivot], A[col]
        b[col], b[pivot] = b[pivot], b[col]
        piv_val = A[col][col]
        for j in range(col, 3):
            A[col][j] /= piv_val
        b[col] /= piv_val
        for row in range(3):
            if row == col:
                continue
            factor = A[row][col]
            for j in range(col, 3):
                A[row][j] -= factor * A[col][j]
            b[row] -= factor * b[col]
    return b[0], b[1], b[2]


def fit_circle_kasa(xs, ys):
    """
    Kasa 最小二乘圆拟合.
    x² + y² = 2·cx·x + 2·cy·y + c0
    返回 (cx, cy, radius, rms_geom)
    """
    n = len(xs)
    if n < 3:
        raise RuntimeError(f"fit_circle_kasa: need ≥3 points, got {n}")

    ATA = [[0.0] * 3 for _ in range(3)]
    ATb = [0.0] * 3
    for x, y in zip(xs, ys):
        a0, a1, a2 = 2.0 * x, 2.0 * y, 1.0
        rhs = x * x + y * y
        ATA[0][0] += a0 * a0
        ATA[0][1] += a0 * a1
        ATA[0][2] += a0 * a2
        ATA[1][0] += a1 * a0
        ATA[1][1] += a1 * a1
        ATA[1][2] += a1 * a2
        ATA[2][0] += a2 * a0
        ATA[2][1] += a2 * a1
        ATA[2][2] += a2 * a2
        ATb[0] += a0 * rhs
        ATb[1] += a1 * rhs
        ATb[2] += a2 * rhs

    cx, cy, c0 = solve_3x3(ATA, ATb)
    r_sq = cx * cx + cy * cy + c0
    radius = math.sqrt(max(0.0, r_sq))

    if radius > 1e-12:
        residuals = [abs(math.hypot(x - cx, y - cy) - radius) for x, y in zip(xs, ys)]
    else:
        residuals = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(r * r for r in residuals) / n)
    return cx, cy, radius, rms


def mad_reject_mean(values, threshold=3.0):
    """MAD 离群点剔除 → (mean, std, n_kept, reject_pct)。"""
    n = len(values)
    if n == 0:
        return 0.0, 0.0, 0, 100.0
    sv = sorted(values)
    median = sv[n // 2]
    mad = sorted(abs(v - median) for v in values)[n // 2]
    sigma = 1.4826 * max(mad, 1e-12)
    kept = [v for v in values if abs(v - median) <= threshold * sigma]
    if len(kept) < 3:
        m = sum(values) / n
        vv = sum((x - m) ** 2 for x in values) / n
        return m, math.sqrt(vv), n, 0.0
    m = sum(kept) / len(kept)
    vv = sum((x - m) ** 2 for x in kept) / len(kept)
    return m, math.sqrt(vv), len(kept), (1.0 - len(kept) / n) * 100.0


def unwrap_yaw(series):
    """将 yaw 序列解包为连续角度。"""
    if not series:
        return []
    out = [series[0]]
    for i in range(1, len(series)):
        d = series[i] - series[i - 1]
        if d > math.pi:
            out.append(out[-1] + d - 2 * math.pi)
        elif d < -math.pi:
            out.append(out[-1] + d + 2 * math.pi)
        else:
            out.append(out[-1] + d)
    return out


# ==================== 融合节点 ====================

class CalibratorVisualizer(Node):
    """旋转中心标定 + 实时可视化节点。"""

    def __init__(self):
        super().__init__('calibrator_visualizer')
        self.stop_event = threading.Event()
        self._lock = threading.Lock()

        # ---- 校准数据（重采样后） ----
        self.samples: List[PoseSample] = []
        self._last_sample_stamp = 0.0
        self._last_status_stamp = 0.0
        self._start_stamp = time.monotonic()

        # ---- 可视化数据（每帧更新） ----
        self.t = deque()
        self.px, self.py, self.pz = deque(), deque(), deque()
        self.roll, self.pitch, self.yaw = deque(), deque(), deque()
        self.vx, self.vy, self.vz = deque(), deque(), deque()
        self.wx, self.wy, self.wz = deque(), deque(), deque()

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_callback, QoSProfile(depth=5000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._print_intro()

    def _print_intro(self):
        print()
        print('  ╔' + '═' * 66 + '╗')
        print('  ║    旋转中心标定 + 实时可视化                             ║')
        print('  ║    /odin1/odometry_highfreq                             ║')
        print('  ╠' + '═' * 66 + '╣')
        print('  ║  ① 遥控车子原地自转（建议 ω≤0.5 rad/s）                 ║')
        print('  ║  ② 从 XY 轨迹图判断旋转质量（是否接近正圆）              ║')
        print('  ║  ③ 多转几圈，Ctrl+C 输出标定结果                       ║')
        print('  ╚' + '═' * 66 + '╝')
        print()

    # ---- 回调 ----

    def _trim_old(self, now_ts):
        min_ts = now_ts - PLOT_WINDOW_SEC
        while self.t and self.t[0] < min_ts:
            self.t.popleft(); self.px.popleft(); self.py.popleft(); self.pz.popleft()
            self.roll.popleft(); self.pitch.popleft(); self.yaw.popleft()
            self.vx.popleft(); self.vy.popleft(); self.vz.popleft()
            self.wx.popleft(); self.wy.popleft(); self.wz.popleft()

    def _odom_callback(self, msg: Odometry):
        now_ts = time.time()

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        tw = msg.twist.twist

        roll, pitch, yaw = quat_to_rpy(
            float(o.x), float(o.y), float(o.z), float(o.w)
        )

        # ---- 可视化（每帧） ----
        with self._lock:
            self.t.append(now_ts)
            self.px.append(float(p.x))
            self.py.append(float(p.y))
            self.pz.append(float(p.z))
            self.roll.append(roll)
            self.pitch.append(pitch)
            self.yaw.append(yaw)
            self.vx.append(float(tw.linear.x))
            self.vy.append(float(tw.linear.y))
            self.vz.append(float(tw.linear.z))
            self.wx.append(float(tw.angular.x))
            self.wy.append(float(tw.angular.y))
            self.wz.append(float(tw.angular.z))
            self._trim_old(now_ts)

        # ---- 校准（重采样） ----
        if now_ts - self._last_sample_stamp >= RESAMPLE_INTERVAL:
            self._last_sample_stamp = now_ts
            self.samples.append(PoseSample(
                x=float(p.x), y=float(p.y), z=float(p.z),
                qx=float(o.x), qy=float(o.y), qz=float(o.z), qw=float(o.w),
                stamp=now_ts,
            ))

        # ---- 终端状态 ----
        if now_ts - self._last_status_stamp >= STATUS_INTERVAL:
            self._last_status_stamp = now_ts
            self._print_live_status(now_ts)

    def _print_live_status(self, now_ts):
        elapsed = now_ts - self._start_stamp
        n = len(self.samples)
        sys.stdout.write(f'\r  [录制] 样本:{n:>6d}  历时:{elapsed:>5.0f}s  |  Ctrl+C 结束')
        sys.stdout.flush()

    # ---- 可视化快照 ----

    def snapshot(self):
        with self._lock:
            return {
                't': list(self.t),
                'px': list(self.px), 'py': list(self.py), 'pz': list(self.pz),
                'roll': list(self.roll), 'pitch': list(self.pitch), 'yaw': list(self.yaw),
                'vx': list(self.vx), 'vy': list(self.vy), 'vz': list(self.vz),
                'wx': list(self.wx), 'wy': list(self.wy), 'wz': list(self.wz),
            }

    # ---- 校准计算 ----

    def compute_and_print(self):
        n = len(self.samples)
        elapsed = time.monotonic() - self._start_stamp
        print()
        print()

        if n < MIN_POINTS:
            print(f'  [错误] 采样点不足: {n} < {MIN_POINTS}')
            return

        xs = [s.x for s in self.samples]
        ys = [s.y for s in self.samples]
        zs = [s.z for s in self.samples]

        # 圆拟合
        cx, cy, radius_fit, rms_geom = fit_circle_kasa(xs, ys)
        cz = sum(zs) / len(zs)

        radii = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
        r_avg = sum(radii) / len(radii)
        r_std = math.sqrt(sum((r - r_avg) ** 2 for r in radii) / len(radii))
        r_std_pct = r_std / max(r_avg, 1e-6) * 100.0

        # 逐帧 offset
        txs_raw, tys_raw, tzs_raw = [], [], []
        roll_list, pitch_list = [], []
        for s in self.samples:
            R = quat_to_rot_matrix(s.qx, s.qy, s.qz, s.qw)
            RT = mat_transpose_3x3(R)
            v_odom = (cx - s.x, cy - s.y, cz - s.z)
            v_odin = mat_vec_mul_3x3(RT, v_odom)
            txs_raw.append(v_odin[0])
            tys_raw.append(v_odin[1])
            tzs_raw.append(v_odin[2])
            r, p, _ = quat_to_rpy(s.qx, s.qy, s.qz, s.qw)
            roll_list.append(r)
            pitch_list.append(p)

        # MAD 剔除
        tx, tx_std, tx_n, tx_rej = mad_reject_mean(txs_raw, OUTLIER_MAD_THRESHOLD)
        ty, ty_std, ty_n, ty_rej = mad_reject_mean(tys_raw, OUTLIER_MAD_THRESHOLD)
        tz, tz_std, tz_n, tz_rej = mad_reject_mean(tzs_raw, OUTLIER_MAD_THRESHOLD)

        # roll/pitch
        roll_avg = sum(roll_list) / len(roll_list)
        pitch_avg = sum(pitch_list) / len(pitch_list)
        roll_std = math.sqrt(sum((r - roll_avg) ** 2 for r in roll_list) / len(roll_list))
        pitch_std = math.sqrt(sum((p - pitch_avg) ** 2 for p in pitch_list) / len(pitch_list))
        roll_out = -roll_avg if USE_PITCH_ROLL_COMP else 0.0
        pitch_out = -pitch_avg if USE_PITCH_ROLL_COMP else 0.0

        # 打印
        fmt = f'{{:.{PRINT_DECIMALS}f}}'
        qual = '✓ 良好' if rms_geom < 0.02 else ('△ 一般' if rms_geom < 0.05 else '✗ 较差')
        print('  ╔' + '═' * 66 + '╗')
        print('  ║                      标 定 结 果                       ║')
        print('  ╠' + '═' * 66 + '╣')
        print(f'  ║  样本: {n}  |  录制: {elapsed:.0f}s')
        print(f'  ║  圆心: cx={fmt.format(cx)}  cy={fmt.format(cy)}  cz={cz:.4f}')
        print(f'  ║  半径: {r_avg:.4f}m  σ={r_std:.4f} ({r_std_pct:.1f}%)  |  残差RMS={rms_geom:.4f}m [{qual}]')
        print(f'  ║  Tx={fmt.format(tx)}±{tx_std:.{PRINT_DECIMALS}f}  Ty={fmt.format(ty)}±{ty_std:.{PRINT_DECIMALS}f}  Tz={fmt.format(tz)}±{tz_std:.{PRINT_DECIMALS}f}')
        print(f'  ║  剔除: tx={tx_rej:.0f}%  ty={ty_rej:.0f}%  tz={tz_rej:.0f}%')
        print(f'  ║  roll={math.degrees(roll_avg):+.2f}°  pitch={math.degrees(pitch_avg):+.2f}°')
        print('  ╠' + '═' * 66 + '╣')
        print('  ║  >>> static_transform_publisher <<<')
        print(f'  ║   --frame-id odin1_base_link --child-frame-id base_link')
        print(f'  ║   --x {fmt.format(tx)} --y {fmt.format(ty)} --z {fmt.format(tz)}')
        print(f'  ║   --roll {fmt.format(roll_out)} --pitch {fmt.format(pitch_out)} --yaw 0.0')
        print('  ╚' + '═' * 66 + '╝')
        print()

        # 在 XY 图上叠加拟合圆
        self._draw_fitted_circle(cx, cy, r_avg, rms_geom)

    def _draw_fitted_circle(self, cx, cy, r, rms):
        """在 XY 图上画拟合圆（需先阻止 matplotlib 关闭）。"""
        try:
            fig = plt.gcf()
            if not plt.get_fignums():
                return
            ax = fig.axes[0]
            theta = [2 * math.pi * i / 200 for i in range(201)]
            circ_x = [cx + r * math.cos(t) for t in theta]
            circ_y = [cy + r * math.sin(t) for t in theta]
            ax.plot(circ_x, circ_y, '--', color='#e74c3c', linewidth=2,
                    label=f'fit circle (r={r:.3f}m, rms={rms:.3f}m)')
            ax.plot(cx, cy, 'x', color='#e74c3c', markersize=12, markeredgewidth=2, label='center')
            ax.legend()
            ax.set_title(f'XY trajectory — samples: {len(self.samples)}  fit-rms: {rms:.4f}m')
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.5)
        except Exception:
            pass


# ==================== 绘图窗口 ====================

def set_ylim(ax, series_list, min_span=0.01):
    vals = [v for s in series_list for v in s]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    span = max(min_span, hi - lo)
    margin = 0.18 * span
    ax.set_ylim(lo - margin, hi + margin)


def run_plot_window(node: CalibratorVisualizer):
    """主线程运行 matplotlib 窗口。"""
    fig, axes = plt.subplots(5, 1, figsize=FIG_SIZE, sharex=False)
    fig.canvas.manager.set_window_title(WINDOW_TITLE)

    # === 子图 0: XY trajectory（最关键：判断是否为圆） ===
    ax_xy = axes[0]
    xy_line, = ax_xy.plot([], [], color='#2c3e50', linewidth=1.0, alpha=0.7, label='odin1 xy')
    ax_xy.set_ylabel('y (m)')
    ax_xy.set_xlabel('x (m)')
    ax_xy.set_title('XY Trajectory (should be a circle)')
    ax_xy.grid(True, alpha=GRID_ALPHA)
    ax_xy.legend(loc='upper left')
    ax_xy.set_aspect('equal', adjustable='box')

    # === 子图 1: Position ===
    ax_pos = axes[1]
    px_l, = ax_pos.plot([], [], color=COLORS['x'], lw=LINE_WIDTH, label='x')
    py_l, = ax_pos.plot([], [], color=COLORS['y'], lw=LINE_WIDTH, label='y')
    pz_l, = ax_pos.plot([], [], color=COLORS['z'], lw=LINE_WIDTH, label='z')
    ax_pos.set_ylabel('position (m)')
    ax_pos.set_title('Position')
    ax_pos.grid(True, alpha=GRID_ALPHA)
    ax_pos.legend(loc='upper left')

    # === 子图 2: Orientation RPY ===
    ax_rpy = axes[2]
    roll_l, = ax_rpy.plot([], [], color=COLORS['x'], lw=LINE_WIDTH, label='roll')
    pitch_l, = ax_rpy.plot([], [], color=COLORS['y'], lw=LINE_WIDTH, label='pitch')
    yaw_l, = ax_rpy.plot([], [], color=COLORS['z'], lw=LINE_WIDTH, label='yaw (unwrapped)')
    ax_rpy.set_ylabel('orientation (rad)')
    ax_rpy.set_title('Orientation')
    ax_rpy.grid(True, alpha=GRID_ALPHA)
    ax_rpy.legend(loc='upper left')

    # === 子图 3: Linear velocity ===
    ax_lin = axes[3]
    vx_l, = ax_lin.plot([], [], color=COLORS['x'], lw=LINE_WIDTH, label='vx')
    vy_l, = ax_lin.plot([], [], color=COLORS['y'], lw=LINE_WIDTH, label='vy')
    vz_l, = ax_lin.plot([], [], color=COLORS['z'], lw=LINE_WIDTH, label='vz')
    ax_lin.set_ylabel('linear (m/s)')
    ax_lin.set_title('Linear Velocity')
    ax_lin.grid(True, alpha=GRID_ALPHA)
    ax_lin.legend(loc='upper left')

    # === 子图 4: Angular velocity ===
    ax_ang = axes[4]
    wx_l, = ax_ang.plot([], [], color=COLORS['x'], lw=LINE_WIDTH, label='wx')
    wy_l, = ax_ang.plot([], [], color=COLORS['y'], lw=LINE_WIDTH, label='wy')
    wz_l, = ax_ang.plot([], [], color=COLORS['z'], lw=LINE_WIDTH, label='wz')
    ax_ang.set_ylabel('angular (rad/s)')
    ax_ang.set_xlabel('time (s)')
    ax_ang.set_title('Angular Velocity')
    ax_ang.grid(True, alpha=GRID_ALPHA)
    ax_ang.legend(loc='upper left')

    plt.tight_layout(rect=(0, 0, 1, 0.99))
    plt.show(block=False)

    def _on_close(_event):
        node.stop_event.set()
    fig.canvas.mpl_connect('close_event', _on_close)

    sleep_dt = 1.0 / max(1.0, PLOT_REFRESH_HZ)

    try:
        while (not node.stop_event.is_set()) and plt.get_fignums():
            snap = node.snapshot()
            if snap['t']:
                t0 = snap['t'][0]
                tr = [ts - t0 for ts in snap['t']]
                window_end = max(PLOT_WINDOW_SEC, tr[-1]) if tr else PLOT_WINDOW_SEC
                window_start = max(0.0, window_end - PLOT_WINDOW_SEC)

                # XY trajectory
                xy_line.set_data(snap['px'], snap['py'])
                # 保持 XY 等比例
                xs_all, ys_all = snap['px'], snap['py']
                if xs_all and ys_all:
                    xlo, xhi = min(xs_all), max(xs_all)
                    ylo, yhi = min(ys_all), max(ys_all)
                    half = max(xhi - xlo, yhi - ylo, 0.01) / 2
                    xmid, ymid = (xlo + xhi) / 2, (ylo + yhi) / 2
                    ax_xy.set_xlim(xmid - half, xmid + half)
                    ax_xy.set_ylim(ymid - half, ymid + half)

                # Position
                px_l.set_data(tr, snap['px'])
                py_l.set_data(tr, snap['py'])
                pz_l.set_data(tr, snap['pz'])

                # Orientation + unwrapped yaw
                yaw_u = unwrap_yaw(snap['yaw'])
                roll_l.set_data(tr, snap['roll'])
                pitch_l.set_data(tr, snap['pitch'])
                yaw_l.set_data(tr, yaw_u)

                # Linear
                vx_l.set_data(tr, snap['vx'])
                vy_l.set_data(tr, snap['vy'])
                vz_l.set_data(tr, snap['vz'])

                # Angular
                wx_l.set_data(tr, snap['wx'])
                wy_l.set_data(tr, snap['wy'])
                wz_l.set_data(tr, snap['wz'])

                # 时间轴
                for ax in axes[1:]:
                    ax.set_xlim(window_start, window_end)

                # Y 轴自适应
                set_ylim(ax_pos, [snap['px'], snap['py'], snap['pz']], 0.01)
                set_ylim(ax_rpy, [snap['roll'], snap['pitch'], yaw_u], 0.1)
                set_ylim(ax_lin, [snap['vx'], snap['vy'], snap['vz']], 0.05)
                set_ylim(ax_ang, [snap['wx'], snap['wy'], snap['wz']], 0.05)

                # 标题显示采样数
                ns = len(node.samples)
                ax_xy.set_title(f'XY Trajectory — calibration samples: {ns}')

            # 被动更新：不进入嵌套事件循环，避免窗口抢夺焦点
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                pass
            time.sleep(sleep_dt)
    except KeyboardInterrupt:
        node.get_logger().info('keyboard interrupt, closing...')
    finally:
        node.stop_event.set()
        plt.close('all')


# ==================== 入口 ====================

def main():
    rclpy.init()
    node = CalibratorVisualizer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    spin_thread.start()

    prev_sigint = signal.getsignal(signal.SIGINT)
    def _sigint(signum, frame):
        if not node.stop_event.is_set():
            node.get_logger().info('SIGINT, shutting down...')
        node.stop_event.set()
        plt.close('all')
    signal.signal(signal.SIGINT, _sigint)

    try:
        run_plot_window(node)
    finally:
        node.stop_event.set()
        try:
            node.compute_and_print()
        except Exception as e:
            print(f'\n[ERROR] {e}')
            import traceback
            traceback.print_exc()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        signal.signal(signal.SIGINT, prev_sigint)


if __name__ == '__main__':
    main()
