#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3-axis chassis speed calibration tool (session-based)

Data sources:
  - Command: /cmd_vel              (geometry_msgs/Twist)
  - Actual:  /odin1/odometry_highfreq (nav_msgs/Odometry)
              twist.twist.linear.x/y  -> linear velocity (odin1_base_link frame)
              twist.twist.angular.z   -> angular velocity

Workflow (3 sessions, switch by key press in figure window):
  Session 1: vx — drive forward/backward only, press [n] to finish
  Session 2: vy — drive left/right only,      press [n] to finish
  Session 3: wz — rotate in place only,       press [f] to finish & output

Usage:
  python3 src/driver/vehicle_driver/script/calib/calib_chassis_speed.py
  # or
  ros2 run vehicle_driver calib_chassis_speed  (if registered in setup.py)

VX/VY/WZ_SCALE are auto-loaded from vehicle_driver/config.py.
CSV and detail logs are saved to the ./logs/ subdirectory next to this script.

Note:
  odin1 velocity is in odin1_base_link frame. If the Odin mounting
  orientation matches the chassis forward direction, linear.x is the
  forward/backward speed directly. Otherwise an extra rotation
  compensation is needed.
"""

from __future__ import annotations

import csv
import math
import os
import queue
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use('Qt5Agg')  # 必须在 import pyplot 之前设置后端
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ---- 配置中文字体，避免 DejaVu Sans 缺 CJK 字形警告 ----
_cjk_fonts = [f for f in fm.findSystemFonts() if 'NotoSansCJK' in f or 'Noto Sans CJK' in f or 'UKai' in f]
if _cjk_fonts:
    _font_prop = fm.FontProperties(fname=_cjk_fonts[0])
    plt.rcParams['font.family'] = _font_prop.get_name()
# ---- 字体配置完成 ----

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ============================================================
# 四元数工具（自包含，不依赖外部包）
# ============================================================
def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """RPY → 单位四元数 (x, y, z, w)"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_multiply(q1: tuple[float, float, float, float],
                   q2: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """四元数乘法 q1 * q2（Hamilton 约定，与 ROS tf2 一致）"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """单位四元数的共轭（= 逆）"""
    return (-q[0], -q[1], -q[2], q[3])


def _quat_rotate(q: tuple[float, float, float, float],
                 v: tuple[float, float, float]) -> tuple[float, float, float]:
    """用四元数旋转向量: v' = q * v * q⁻¹"""
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz)) * v[0] + 2.0 * (xy - wz) * v[1] + 2.0 * (xz + wy) * v[2],
        2.0 * (xy + wz) * v[0] + (1.0 - 2.0 * (xx + zz)) * v[1] + 2.0 * (yz - wx) * v[2],
        2.0 * (xz - wy) * v[0] + 2.0 * (yz + wx) * v[1] + (1.0 - 2.0 * (xx + yy)) * v[2],
    )


def _quat_to_rpy(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """四元数 → roll, pitch, yaw"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


# ============================================================
# Paths
# ============================================================
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_LOG_DIR = os.path.join(_SCRIPT_DIR, 'logs')
_LOG_DETAIL_DIR = os.path.join(_SCRIPT_DIR, 'logs')


# ============================================================
# Auto-load scales from config.py (try import first, fallback file load)
# ============================================================
def _load_config_scales() -> tuple[float, float, float]:
    try:
        from vehicle_driver import config as _vd_config
        return (
            float(getattr(_vd_config, 'VX_SCALE', 660.0)),
            float(getattr(_vd_config, 'VY_SCALE', 660.0)),
            float(getattr(_vd_config, 'WZ_SCALE', 660.0)),
        )
    except (ImportError, ModuleNotFoundError):
        pass
    # Fallback: load via file path (works without workspace sourced)
    import importlib.util
    _fallback_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'vehicle_driver', 'config.py')
    if os.path.exists(_fallback_path):
        spec = importlib.util.spec_from_file_location('_vd_config', _fallback_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return (
            float(getattr(mod, 'VX_SCALE', 660.0)),
            float(getattr(mod, 'VY_SCALE', 660.0)),
            float(getattr(mod, 'WZ_SCALE', 660.0)),
        )
    print(f'[WARN] cannot load config.py, using default 660.0')
    return 660.0, 660.0, 660.0


CUR_VX_SCALE, CUR_VY_SCALE, CUR_WZ_SCALE = _load_config_scales()
CUR_SCALES = [CUR_VX_SCALE, CUR_VY_SCALE, CUR_WZ_SCALE]


# ==================== 话题名 ====================
CMD_TOPIC  = '/cmd_vel'       # 底盘速度指令话题
ODOM_TOPIC = '/Odometry'       # 里程计话题 (small_point_lio 输出)

# ==================== 订阅 QoS ====================
SUB_QOS_DEPTH = 200            # cmd_vel 订阅队列深度

# ==================== 可视化参数 ====================
PLOT_WINDOW_SEC = 20.0         # 实时图窗时间范围 (秒)
PLOT_REFRESH_HZ = 20.0         # 图窗刷新频率 (Hz)
FIG_SIZE        = (12, 8)      # 图窗尺寸 (英寸)
GRID_ALPHA      = 0.28         # 网格透明度
LINE_WIDTH      = 1.5          # 曲线线宽
COLOR_CMD       = '#1f77b4'    # 指令速度曲线颜色 (蓝)
COLOR_ODOM      = '#ff7f0e'    # 里程计曲线颜色 (橙)
COLOR_HL_BG     = '#fffbe6'    # 高亮背景色

# ==================== 标定拟合参数 ====================
FIT_DEADBAND_XY = 0.01         # XY 线性速度死区 (m/s), 低于此不参与拟合
FIT_DEADBAND_WZ = 0.02         # 角速度死区 (rad/s)
MAD_SIGMA       = 3.5          # 异常值剔除: |残差| > sigma * MAD_SIGMA

# ==================== 里程计外参 (odom -> base_link) ====================
ODIN_TO_BASE_XYZ = (-0.268846, 0.028504, 0.067861)  # 平移 (m)
ODIN_TO_BASE_RPY = (0.017828, 0.245433, 0.0)        # 旋转 (roll, pitch, yaw rad)

# ==================== 标定流程 ====================
SESSIONS = [
    {'name': 'vx', 'label': 'vx linear-x [m/s]',       'unit': 'm/s',   'deadband': FIT_DEADBAND_XY},
    {'name': 'vy', 'label': 'vy linear-y [m/s]',       'unit': 'm/s',   'deadband': FIT_DEADBAND_XY},
    {'name': 'wz', 'label': 'wz angular-z [rad/s]',    'unit': 'rad/s', 'deadband': FIT_DEADBAND_WZ},
]

SESSION_HINTS = [
    '第 1/3 轮: 标定 vx (前后) — 前进/后退   | 按 [n] 完成本轮',
    '第 2/3 轮: 标定 vy (左右) — 左移/右移   | 按 [n] 完成本轮',
    '第 3/3 轮: 标定 wz (旋转) — 原地旋转   | 按 [f] 结束并输出结果',
]


# ============================================================
# Fitting utilities
# ============================================================
def robust_scale_fit(cmd: np.ndarray, odom: np.ndarray) -> tuple[float, float]:
    """Robust origin-passing least squares, returns (slope, kept_ratio)."""
    denom = float(np.dot(cmd, cmd))
    if denom <= 1e-12:
        return float('nan'), 0.0
    slope0 = float(np.dot(cmd, odom) / denom)
    resid  = odom - slope0 * cmd
    med    = float(np.median(resid))
    mad    = float(np.median(np.abs(resid - med)))
    if mad > 1e-12:
        mask = np.abs(resid - med) <= MAD_SIGMA * 1.4826 * mad
    else:
        mask = np.ones(len(cmd), dtype=bool)
    kept = int(np.sum(mask))
    if kept < 5:
        return slope0, kept / max(1, len(cmd))
    d2 = float(np.dot(cmd[mask], cmd[mask]))
    slope = float(np.dot(cmd[mask], odom[mask]) / d2) if d2 > 1e-12 else slope0
    return slope, kept / len(cmd)


def analyse_session(
    pairs: list[tuple[float, float]],
    cur_scale: float,
    deadband: float,
) -> dict | None:
    """Time-aligned (cmd, odom) pairs → forced-origin least-squares fit."""
    n = len(pairs)
    if n < 10:
        return None
    cmd  = np.array([p[0] for p in pairs])
    odom = np.array([p[1] for p in pairs])
    mask = np.abs(cmd) > deadband
    if int(np.sum(mask)) < 5:
        return None
    slope, kept_ratio = robust_scale_fit(cmd[mask], odom[mask])
    if not np.isfinite(slope) or abs(slope) < 1e-4:
        return None
    recommended = cur_scale / slope
    return {
        'n_total'          : n,
        'n_fit'            : int(np.sum(mask)),
        'slope'            : slope,
        'kept_ratio'       : kept_ratio,
        'recommended_scale': recommended,
        'delta_pct'        : (recommended - cur_scale) / cur_scale * 100.0,
        'mae'              : float(np.mean(np.abs(odom[mask] - slope * cmd[mask]))),
    }


# ============================================================
# ROS Node
# ============================================================
class ChassisCalibNode(Node):

    def __init__(self):
        super().__init__('chassis_calib_node')
        self.stop_event = threading.Event()
        self._lock = threading.Lock()

        self._session_idx     = 0
        self._session_results : list[dict | None] = [None, None, None]

        # Last known cmd value (realtime pairing with odom in _odom_cb)
        self._last_cmd = [0.0, 0.0, 0.0]  # [vx, vy, wz]

        # Time-aligned (cmd, odom) pairs for fitting: _odom_cb pairs each
        # odom measurement with the latest cmd value at that moment.
        self._session_pairs: list[deque] = [deque() for _ in range(3)]

        # Rolling plot buffers: cmd and odom have independent time axes
        self._cmd_buf  = [{'t': deque(), 'v': deque()} for _ in range(3)]
        self._odom_buf = [{'t': deque(), 'v': deque()} for _ in range(3)]

        # CSV
        os.makedirs(_LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(_LOG_DIR, f'chassis_calib_{ts}.csv')
        self._csv_q    = queue.Queue(maxsize=50000)
        self._csv_thread = threading.Thread(
            target=self._csv_loop, daemon=True, name='csv_writer')
        self._init_csv()
        self._csv_thread.start()

        self.create_subscription(Twist,    CMD_TOPIC,  self._cmd_cb,  SUB_QOS_DEPTH)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_cb,
            QoSProfile(depth=5000, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info(
            f'Subscribed: {CMD_TOPIC}  |  {ODOM_TOPIC}')
        self.get_logger().info(
            f'config.py scale: VX={CUR_VX_SCALE:.1f}  VY={CUR_VY_SCALE:.1f}  WZ={CUR_WZ_SCALE:.1f}')
        self.get_logger().info(f'CSV -> {self._csv_path}')

        # 详细 odom 日志（所有原始 + 计算参数）
        os.makedirs(_LOG_DETAIL_DIR, exist_ok=True)
        self._detail_path = os.path.join(_LOG_DETAIL_DIR, f'odom_detail_{ts}.csv')
        self._detail_q = queue.Queue(maxsize=50000)
        self._detail_thread = threading.Thread(
            target=self._detail_loop, daemon=True, name='odom_detail_writer')
        self._init_detail_csv()
        self._detail_thread.start()
        self.get_logger().info(f'详细里程计日志 -> {self._detail_path}')

    # ---- CSV ----
    def _init_csv(self):
        with open(self._csv_path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(
                ['recv_time_sec', 'source', 'session', 'vx', 'vy', 'wz'])

    def _csv_loop(self):
        with open(self._csv_path, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            while not self.stop_event.is_set() or not self._csv_q.empty():
                try:
                    w.writerow(self._csv_q.get(timeout=0.2))
                    f.flush()
                except queue.Empty:
                    continue

    def _csv_put(self, row):
        try:
            self._csv_q.put_nowait(row)
        except queue.Full:
            pass

    # ---- 详细里程计日志（多列 CSV，所有原始 + 计算参数） ----
    _DETAIL_HEADER = [
        'recv_time_sec',
        # ROS header stamp
        'stamp_sec', 'stamp_nanosec',
        # 原始 pose
        'pose_x', 'pose_y', 'pose_z',
        'orient_x', 'orient_y', 'orient_z', 'orient_w',
        # 原始 twist
        'twist_lin_x', 'twist_lin_y', 'twist_lin_z',
        'twist_ang_x', 'twist_ang_y', 'twist_ang_z',
        # pose covariance 对角元
        'cov_xx', 'cov_yy', 'cov_zz', 'cov_aa',
        # 变换后 base_link 位置（odom 系）
        'base_x', 'base_y', 'base_z',
        # 变换后 base_link 朝向
        'base_qx', 'base_qy', 'base_qz', 'base_qw',
        # 计算速度（base_link 系，twist + 外参旋转）
        'computed_vx', 'computed_vy', 'computed_wz',
        # 配对的 cmd（_last_cmd 当时值）
        'paired_cmd_vx', 'paired_cmd_vy', 'paired_cmd_wz',
    ]

    def _init_detail_csv(self):
        with open(self._detail_path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(self._DETAIL_HEADER)

    def _detail_loop(self):
        with open(self._detail_path, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            while not self.stop_event.is_set() or not self._detail_q.empty():
                try:
                    w.writerow(self._detail_q.get(timeout=0.2))
                    f.flush()
                except queue.Empty:
                    continue

    def _detail_put(self, row):
        try:
            self._detail_q.put_nowait(row)
        except queue.Full:
            pass

    # ---- Topic callbacks ----
    def _cmd_cb(self, msg: Twist):
        now = time.time()
        vx, vy, wz = float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
        with self._lock:
            self._last_cmd = [vx, vy, wz]
            si = self._session_idx
            self._push_buf(self._cmd_buf[si], now, [vx, vy, wz][si])
        self._csv_put([now, 'cmd', si, vx, vy, wz])

    def _odom_cb(self, msg: Odometry):
        now = time.time()

        # 从话题读取 odin1 在 odom 系下的位置和朝向
        p_odin = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        q_odom_odin1 = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )

        # 用外参将 odin1 位置变换到 base_link 位置（在 odom 系下）
        # p_base_odom = p_odin_odom + R(q_odom_odin1) * t_odin_base
        t_odin_base = ODIN_TO_BASE_XYZ
        offset = _quat_rotate(q_odom_odin1, t_odin_base)
        p_base = (p_odin[0] + offset[0], p_odin[1] + offset[1], p_odin[2] + offset[2])

        # 合成 base_link 朝向: q_odom_base = q_odom_odin1 * q_odin1_base
        q_odin1_base = _quat_from_rpy(*ODIN_TO_BASE_RPY)
        q_odom_base = _quat_multiply(q_odom_odin1, q_odin1_base)

        # === 从 twist 计算速度（比 pose 微分平滑 1-2 个数量级）===
        # 读取 odin1 系下的线速度和角速度
        vx_odin1 = float(msg.twist.twist.linear.x)
        vy_odin1 = float(msg.twist.twist.linear.y)
        vz_odin1 = float(msg.twist.twist.linear.z)
        wx_odin1 = float(msg.twist.twist.angular.x)
        wy_odin1 = float(msg.twist.twist.angular.y)
        wz_odin1 = float(msg.twist.twist.angular.z)

        # 刚体修正: 因为 base_link 不在 odin1 原点，速度需要加 ω × t 项
        # v_at_base = v_odin1 + ω_odin1 × t_odin_base
        tx, ty, tz = ODIN_TO_BASE_XYZ
        v_corr_x = wy_odin1 * tz - wz_odin1 * ty
        v_corr_y = wz_odin1 * tx - wx_odin1 * tz
        v_corr_z = wx_odin1 * ty - wy_odin1 * tx

        v_odin1_at_base = (vx_odin1 + v_corr_x, vy_odin1 + v_corr_y, vz_odin1 + v_corr_z)
        w_odin1_vec = (wx_odin1, wy_odin1, wz_odin1)

        # 转到 base_link 系: q_odin1_base 是 odin1 系 → base_link 系的旋转
        v_base = _quat_rotate(q_odin1_base, v_odin1_at_base)
        w_base = _quat_rotate(q_odin1_base, w_odin1_vec)

        vx = v_base[0]
        vy = v_base[1]
        wz = w_base[2]

        # 存入 session buffer（实时配对：odom 与最近一次 cmd）
        vals = [vx, vy, wz]
        with self._lock:
            si = self._session_idx
            cmd_val = self._last_cmd[si]
            last_cmd_snapshot = list(self._last_cmd)  # 全部 3 轴，供详细日志
            self._push_buf(self._odom_buf[si], now, vals[si])
            self._session_pairs[si].append((cmd_val, vals[si]))
        self._csv_put([now, 'odom', si, vx, vy, wz])

        # ---- 详细日志（每帧写入所有原始 + 计算参数） ----
        pcov = msg.pose.covariance
        detail_row = [
            now,
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            p_odin[0], p_odin[1], p_odin[2],
            q_odom_odin1[0], q_odom_odin1[1], q_odom_odin1[2], q_odom_odin1[3],
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
            float(msg.twist.twist.angular.x),
            float(msg.twist.twist.angular.y),
            float(msg.twist.twist.angular.z),
            float(pcov[0]) if len(pcov) > 0 else 0.0,
            float(pcov[7]) if len(pcov) > 7 else 0.0,
            float(pcov[14]) if len(pcov) > 14 else 0.0,
            float(pcov[35]) if len(pcov) > 35 else 0.0,
            p_base[0], p_base[1], p_base[2],
            q_odom_base[0], q_odom_base[1], q_odom_base[2], q_odom_base[3],
            vx, vy, wz,
            last_cmd_snapshot[0], last_cmd_snapshot[1], last_cmd_snapshot[2],
        ]
        self._detail_put(detail_row)

    def _push_buf(self, buf: dict, t: float, v: float):
        buf['t'].append(t)
        buf['v'].append(v)
        min_t = t - PLOT_WINDOW_SEC
        while buf['t'] and buf['t'][0] < min_t:
            buf['t'].popleft()
            buf['v'].popleft()

    # ---- Session management ----
    def next_session(self) -> int:
        """Analyse current session and advance to next; returns completed session idx."""
        with self._lock:
            si = self._session_idx
            pairs = list(self._session_pairs[si])
        result = analyse_session(
            pairs, CUR_SCALES[si], SESSIONS[si]['deadband'])
        with self._lock:
            self._session_results[si] = result
            if si < 2:
                self._session_idx = si + 1
        self._print_session(si, result)
        return si

    def reset_current_session(self) -> None:
        """清空当前 session 的所有已采集数据，方便重新标定。"""
        with self._lock:
            si = self._session_idx
            name = SESSIONS[si]['name']
            self._session_pairs[si].clear()
            self._cmd_buf[si]['t'].clear()
            self._cmd_buf[si]['v'].clear()
            self._odom_buf[si]['t'].clear()
            self._odom_buf[si]['v'].clear()
            self._session_results[si] = None
        print(f'  [重置] 第 {si + 1}/3 轮 ({name}) 数据已清空，可以重新采集')

    def finish(self):
        """Analyse last session and print summary."""
        with self._lock:
            si = self._session_idx
            pairs = list(self._session_pairs[si])
        result = analyse_session(
            pairs, CUR_SCALES[si], SESSIONS[si]['deadband'])
        with self._lock:
            self._session_results[si] = result
        self._print_session(si, result)
        self._print_summary()
        self.stop_event.set()

    # ---- Printing ----
    def _print_session(self, si: int, r: dict | None):
        name_cn = ['前后(vx)', '左右(vy)', '旋转(wz)'][si]
        unit = SESSIONS[si]['unit']
        print(f'\n{"="*56}')
        print(f'  第 {si+1}/3 轮: {name_cn} 标定结果')
        print(f'{"="*56}')
        if r is None:
            print('  [警告] 有效命令样本不足 5 个，无法拟合。')
            print('  请重新采集（确保发出明确的速度指令）。')
        else:
            print(f'  样本数: 总计 {r["n_total"]}，有效 {r["n_fit"]}')
            print(f'  斜率:   {r["slope"]:.6f}  (MAD 保留 {r["kept_ratio"]*100:.1f}%)')
            print(f'  平均绝对误差: {r["mae"]:.5f} {unit}')
            print(f'  当前 scale:   {CUR_SCALES[si]:.2f}')
            print(f'  建议值:       {r["recommended_scale"]:.2f}  ({r["delta_pct"]:+.2f}%)')
        print()

    def _print_summary(self):
        names = ['VX_SCALE', 'VY_SCALE', 'WZ_SCALE']
        name_cn = ['前后(vx)', '左右(vy)', '旋转(wz)']
        print(f'\n{"="*56}')
        print(f'  标 定 总 结')
        print(f'  时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        print(f'  CSV : {self._csv_path}')
        print(f'{"="*56}')
        for i, (r, name) in enumerate(zip(self._session_results, names)):
            if r and np.isfinite(r['recommended_scale']):
                val = f'{r["recommended_scale"]:.2f}'
                print(f'  {name_cn[i]} → {name} = {val}')
            else:
                val = f'{CUR_SCALES[i]:.2f}'
                print(f'  {name_cn[i]} → {name} = {val}  # [未标定，保持原值]')
        print(f'  {"="*22}  复制到 config.py  {"="*22}')
        print(f'  # 下面这 3 行可以直接替换 config.py 中对应项:')
        for i, (r, name) in enumerate(zip(self._session_results, names)):
            if r and np.isfinite(r['recommended_scale']):
                val = f'{r["recommended_scale"]:.2f}'
            else:
                val = f'{CUR_SCALES[i]:.2f}'
            print(f'  {name} = {val}')
        print(f'{"="*56}\n')

    # ---- Snapshot for plot thread ----
    def snapshot(self) -> dict:
        with self._lock:
            return {
                'si': self._session_idx,
                'cmd_bufs' : [
                    (list(self._cmd_buf[i]['t']),  list(self._cmd_buf[i]['v']))
                    for i in range(3)],
                'odom_bufs': [
                    (list(self._odom_buf[i]['t']), list(self._odom_buf[i]['v']))
                    for i in range(3)],
            }

    def shutdown(self):
        self.stop_event.set()
        self._csv_thread.join(timeout=3.0)
        self._detail_thread.join(timeout=3.0)


# ============================================================
# Plot main loop (main thread)
# ============================================================
def run_plot_window(node: ChassisCalibNode):
    fig, axes = plt.subplots(3, 1, figsize=FIG_SIZE, sharex=False)
    fig.canvas.manager.set_window_title('Chassis Speed Calibration')

    lines_cmd, lines_odom = [], []
    for i, ax in enumerate(axes):
        lc, = ax.plot([], [], color=COLOR_CMD,  lw=LINE_WIDTH, label='cmd_vel')
        lo, = ax.plot([], [], color=COLOR_ODOM, lw=LINE_WIDTH, label='odom (highfreq)')
        lines_cmd.append(lc)
        lines_odom.append(lo)
        ax.set_ylabel(SESSIONS[i]['label'], fontsize=9)
        ax.grid(True, alpha=GRID_ALPHA)
        ax.legend(loc='upper right', fontsize=8)

    status_txt = fig.text(
        0.5, 0.99, SESSION_HINTS[0],
        ha='center', va='top', fontsize=10, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#e8f4fd', alpha=0.9),
    )
    plt.tight_layout(rect=(0, 0, 1, 0.965))
    plt.show(block=False)

    # ---- Key events (inside matplotlib window) ----
    def _on_key(event):
        if node.stop_event.is_set():
            return
        if event.key == 'n':
            done_si = node.next_session()
            with node._lock:
                new_si = node._session_idx
            if new_si > done_si:
                name_cn = ['前后(vx)', '左右(vy)', '旋转(wz)'][new_si]
                print(f'[切换] → 第 {new_si+1}/3 轮: 标定 {name_cn}')
            elif done_si == 2:
                print('[提示] 已是最后一轮，按 [f] 结束并输出结果')
        elif event.key == 'f':
            node.finish()
        elif event.key == 'c':
            node.reset_current_session()

    fig.canvas.mpl_connect('key_press_event', _on_key)
    fig.canvas.mpl_connect('close_event', lambda _e: node.stop_event.set())

    sleep_dt = 1.0 / max(1.0, PLOT_REFRESH_HZ)

    try:
        while not node.stop_event.is_set() and plt.get_fignums():
            snap = node.snapshot()
            si   = snap['si']

            status_txt.set_text(SESSION_HINTS[min(si, 2)])

            for i, ax in enumerate(axes):
                t_cmd,  v_cmd  = snap['cmd_bufs'][i]
                t_odom, v_odom = snap['odom_bufs'][i]

                ax.set_facecolor(COLOR_HL_BG if i == si else 'white')

                if t_cmd and t_odom:
                    t0 = min(t_cmd[0], t_odom[0])
                    tc = [t - t0 for t in t_cmd]
                    to = [t - t0 for t in t_odom]
                    lines_cmd[i].set_data(tc,  v_cmd)
                    lines_odom[i].set_data(to, v_odom)

                    x_max = max(tc[-1] if tc else 0, to[-1] if to else 0)
                    ax.set_xlim(max(0.0, x_max - PLOT_WINDOW_SEC), x_max + 0.5)

                    all_v = list(v_cmd) + list(v_odom)
                    if all_v:
                        vmin, vmax = min(all_v), max(all_v)
                        span = max(0.05, vmax - vmin)
                        ax.set_ylim(vmin - 0.2*span, vmax + 0.2*span)

            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                pass
            time.sleep(sleep_dt)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        try:
            plt.close('all')
        except Exception:
            pass


# ============================================================
# Main entry
# ============================================================
def main():
    print('=' * 56)
    print('  底盘三轴速度标定工具 (Session-based)')
    print(f'  config.py 当前值: VX={CUR_VX_SCALE:.1f}  VY={CUR_VY_SCALE:.1f}  WZ={CUR_WZ_SCALE:.1f}')
    print(f'  标定日志: {_LOG_DIR}')
    print(f'  详细里程计日志: {_LOG_DETAIL_DIR}/')
    print('=' * 56)
    print('  【操作说明】')
    print('  标定分 3 轮进行，按顺序依次完成：')
    print()
    print('    第 1 轮 — 前后 (vx)')
    print('      遥控车前进/后退，在图窗按 [n] 完成')
    print()
    print('    第 2 轮 — 左右 (vy)')
    print('      遥控车左移/右移，在图窗按 [n] 完成')
    print()
    print('    第 3 轮 — 旋转 (wz)')
    print('      遥控车原地旋转，在图窗按 [f] 结束并输出结果')
    print()
    print('  【快捷键】')
    print('    [n]  完成当前轮，进入下一轮')
    print('    [f]  完成最后一轮并输出标定结果')
    print('    [c]  清空当前轮的已采集数据，重新标定（数据有问题时使用）')
    print()
    print('  【提示】')
    print('    - 每轮确保发出明确的速度指令，避免零值附近晃动')
    print('    - Ctrl+C 或关闭图窗可随时退出（会输出已完成的标定结果）')
    print('=' * 56 + '\n')

    rclpy.init()
    node = ChassisCalibNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=lambda: executor.spin(), daemon=True, name='ros_spin')
    spin_thread.start()

    prev = signal.getsignal(signal.SIGINT)

    def _sigint(s, f):
        node.stop_event.set()
        try:
            plt.close('all')
        except Exception:
            pass

    signal.signal(signal.SIGINT, _sigint)

    try:
        run_plot_window(node)
    finally:
        node.stop_event.set()
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        signal.signal(signal.SIGINT, prev)


if __name__ == '__main__':
    main()
