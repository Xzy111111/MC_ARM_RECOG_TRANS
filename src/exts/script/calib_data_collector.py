#!/usr/bin/env python3
"""
手眼标定数据采集工具 — 全自动模式 (ROS2 + PySide2 + OpenCV)
- 左侧: D435i 相机画面 + ArUco 二维码检测
- 右侧: 6 关节角度卡片 + 自动采集状态
- 自动采集: 检测到 Tag → 读关节角度 → 写 CSV (冷却 1.5s, 关节变化 > 0.02 rad)
- 不保存图像, 纯 CSV
"""

import sys
import os
import time
import csv
import threading
import subprocess
import shutil
from datetime import datetime

# ── ROS2 ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Image, JointState, CameraInfo

# D435i 相机发布 RELIABLE 图像, 订阅必须也用 RELIABLE 才能收到数据
# (qos_profile_sensor_data 是 BEST_EFFORT, 与相机不兼容 → 收不到图)
IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)

# ── OpenCV ────────────────────────────────────────────────────────────────
import cv2
import numpy as np

# ── PySide2 ───────────────────────────────────────────────────────────────
from PySide2.QtCore import Qt, QTimer
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QFrame, QSizePolicy, QSplitter, QTextEdit,
)
from PySide2.QtGui import QPixmap, QImage, QFont, QKeyEvent

# ============================================================================
# 配置
# ============================================================================

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
NUM_JOINTS = 6
CARD_COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12", "#9b59b6", "#1abc9c"]

# ── ArUco ─────────────────────────────────────────────────────────────
# 多词典遍历 — 按常见程度排序, 逐个尝试直到检测到
ARUCO_DICTS = [
    cv2.aruco.DICT_4X4_50,
    cv2.aruco.DICT_5X5_50,
    cv2.aruco.DICT_6X6_50,
    cv2.aruco.DICT_7X7_50,
    cv2.aruco.DICT_ARUCO_ORIGINAL,
]
ARUCO_DICT_NAMES = ["4x4", "5x5", "6x6", "7x7", "ORIGINAL"]

TAG_ID = 12                       # 采集目标的 marker ID
TAG_SIZE = 0.128                  # 标定板边长 (米) — 按实际尺寸改!

# D435i 默认内参
DEFAULT_K = np.array([[610.0, 0, 320.0],
                       [0, 610.0, 240.0],
                       [0, 0, 1.0]], dtype=np.float64)
DEFAULT_D = np.zeros(5, dtype=np.float64)

# 输出目录
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "data")


# ============================================================================
# ROS2 数据缓冲
# ============================================================================

class DataBuffer:
    """线程安全的数据缓冲"""

    def __init__(self):
        self.lock = threading.Lock()

        self.frame_bgr = None
        self.frame_count = 0

        self.joint_positions = [0.0] * NUM_JOINTS

        self.K = DEFAULT_K.copy()
        self.D = DEFAULT_D.copy()
        self.has_camera_info = False

        self.tag_detected = False
        self.tag_tvec = None
        self.tag_rvec = None

    # ── 图像 ──────────────────────────────────────────────────────────

    def put_image(self, msg):
        try:
            bgr = self._rosimg_to_bgr(msg)
        except Exception as e:
            print(f"[图像错误] {e}")
            return
        with self.lock:
            self.frame_bgr = bgr
            self.frame_count += 1
            if self.frame_count == 1:
                print(f"[首帧] {msg.width}x{msg.height} 编码={msg.encoding}")

    def get_image(self):
        with self.lock:
            if self.frame_bgr is None:
                return None
            return self.frame_bgr.copy()

    # ── 关节 ──────────────────────────────────────────────────────────

    def put_joints(self, msg: JointState):
        with self.lock:
            n = min(len(msg.position), NUM_JOINTS)
            for i in range(n):
                self.joint_positions[i] = msg.position[i]

    def get_joints(self):
        with self.lock:
            return list(self.joint_positions)

    # ── 相机内参 ──────────────────────────────────────────────────────

    def put_camera_info(self, msg: CameraInfo):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3).copy()
            self.D = np.array(msg.d).copy()
            self.has_camera_info = True

    def get_camera_params(self):
        with self.lock:
            return self.K.copy(), self.D.copy()

    # ── Tag ───────────────────────────────────────────────────────────

    def put_tag_pose(self, rvec, tvec):
        with self.lock:
            self.tag_detected = True
            self.tag_rvec = rvec.copy()
            self.tag_tvec = tvec.copy()

    def clear_tag(self):
        with self.lock:
            self.tag_detected = False
            self.tag_rvec = None
            self.tag_tvec = None

    def get_tag_pose(self):
        with self.lock:
            if not self.tag_detected:
                return None, None
            return self.tag_rvec.copy(), self.tag_tvec.copy()

    @staticmethod
    def _rosimg_to_bgr(msg):
        if msg.encoding in ("rgb8",):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ("bgr8",):
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        elif msg.encoding in ("mono8", "8UC1"):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width).copy()
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ("bgra",):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4).copy()
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif msg.encoding in ("rgba",):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4).copy()
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding in ("16UC1", "mono16"):
            img = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
            img = (img >> 8).astype(np.uint8)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()


# ============================================================================
# ROS2 订阅
# ============================================================================

class CalibSubscriber(Node):
    def __init__(self, buf: DataBuffer):
        super().__init__("calib_data_collector")
        self.sub_img = self.create_subscription(
            Image, "/camera/camera/color/image_raw", buf.put_image,
            IMG_QOS)
        self.sub_joint = self.create_subscription(
            JointState, "/feedback/joint_states", buf.put_joints, 10)
        self.sub_caminfo = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", buf.put_camera_info, 10)
        self.get_logger().info("自动采集器已启动")


# ============================================================================
# ArUco 检测
# ============================================================================

class ArucoDetector:
    """多词典 ArUco 检测器 — 自动兼容新旧 OpenCV API, 画所有 marker, 只对目标 ID 算位姿"""

    def __init__(self):
        # 检测 OpenCV 版本, 兼容新旧 API
        self._use_new_api = hasattr(cv2.aruco, 'ArucoDetector')

        self.dictionaries = []
        self.detectors = []         # 仅新 API 使用
        self._params = None
        for d in ARUCO_DICTS:
            dictionary = cv2.aruco.getPredefinedDictionary(d)
            self.dictionaries.append(dictionary)
            if self._use_new_api:
                params = cv2.aruco.DetectorParameters()
                det = cv2.aruco.ArucoDetector(dictionary, params)
                self.detectors.append(det)

        if not self._use_new_api:
            # 旧版 OpenCV (4.5.x): 用 DetectorParameters_create
            self._params = cv2.aruco.DetectorParameters_create()

        self._dict_idx = 0
        self._dict_name = "?"

    def detect(self, bgr_img, K, D):
        """
        遍历多词典检测 ArUco marker。
        返回 (corners, ids, target_rvec, target_tvec, dict_name)
        """
        for di, dictionary in enumerate(self.dictionaries):
            try:
                if self._use_new_api:
                    corners, ids, _ = self.detectors[di].detectMarkers(bgr_img)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(
                        bgr_img, dictionary, parameters=self._params)
            except Exception:
                continue
            if ids is not None and len(ids) > 0:
                self._dict_idx = di
                self._dict_name = ARUCO_DICT_NAMES[di]

                # 找目标 ID 的位姿
                target_rvec, target_tvec = None, None
                for i, mid in enumerate(ids.flatten()):
                    if mid == TAG_ID:
                        obj_points = np.array([
                            [-TAG_SIZE/2,  TAG_SIZE/2, 0],
                            [ TAG_SIZE/2,  TAG_SIZE/2, 0],
                            [ TAG_SIZE/2, -TAG_SIZE/2, 0],
                            [-TAG_SIZE/2, -TAG_SIZE/2, 0],
                        ], dtype=np.float64)
                        img_pts = corners[i].reshape(4, 2).astype(np.float64)
                        ok, rv, tv = cv2.solvePnP(obj_points, img_pts, K, D)
                        if ok:
                            target_rvec, target_tvec = rv, tv
                        break

                return corners, ids, target_rvec, target_tvec, self._dict_name

        return None, None, None, None, "?"


# ============================================================================
# 关节角度卡片
# ============================================================================

class JointCard(QFrame):
    def __init__(self, name: str, color: str):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"""
            JointCard {{
                background-color: {color};
                border-radius: 8px;
                padding: 8px;
            }}
        """)
        self.setMinimumSize(120, 75)
        self.setMaximumHeight(85)

        layout = QVBoxLayout(self)
        layout.setSpacing(2)
        layout.setContentsMargins(8, 4, 8, 4)

        self.name_lbl = QLabel(name)
        self.name_lbl.setFont(QFont("Monospace", 12, QFont.Bold))
        self.name_lbl.setStyleSheet("color: white;")
        self.name_lbl.setAlignment(Qt.AlignCenter)

        self.rad_lbl = QLabel("-- rad")
        self.rad_lbl.setFont(QFont("Monospace", 13, QFont.Bold))
        self.rad_lbl.setStyleSheet("color: white;")
        self.rad_lbl.setAlignment(Qt.AlignCenter)

        self.deg_lbl = QLabel("-- °")
        self.deg_lbl.setFont(QFont("Monospace", 10))
        self.deg_lbl.setStyleSheet("color: rgba(255,255,255,0.75);")
        self.deg_lbl.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.name_lbl)
        layout.addWidget(self.rad_lbl)
        layout.addWidget(self.deg_lbl)

    def update_value(self, rad: float):
        deg = rad * 180.0 / 3.1415926535
        self.rad_lbl.setText(f"{rad:+.4f} rad")
        self.deg_lbl.setText(f"{deg:+.1f} °")


# ============================================================================
# 主窗口
# ============================================================================

class MainWindow(QMainWindow):
    def __init__(self, buf: DataBuffer, detector: ArucoDetector):
        super().__init__()
        self.buf = buf
        self.detector = detector

        # 自动采集状态
        self._save_counter = 0
        self._last_save_time = 0.0      # 0.5s 最小间隔防连拍
        self._auto_status_text = "⏸ 等待 Tag..."
        self._csv_path = os.path.join(OUTPUT_DIR, "calib_data.csv")

        self._init_csv()

        self.setWindowTitle("手眼标定数据采集器 — 全自动 Eye-on-Hand")
        self.setMinimumSize(1100, 580)
        self.setStyleSheet("QMainWindow { background-color: #1e1e2e; }")

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # ── 左侧: 相机画面 ───────────────────────────────────────────
        left_panel = QVBoxLayout()
        left_panel.setSpacing(4)

        self.img_label = QLabel("等待图像...")
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setStyleSheet(
            "color: #888; font-size: 16px; background: #121220; border-radius: 6px;")
        self.img_label.setMinimumSize(640, 480)
        self.img_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.tag_status = QLabel("Tag: 未检测")
        self.tag_status.setFont(QFont("Monospace", 10))
        self.tag_status.setStyleSheet("color: #888; padding: 2px;")

        self.img_info = QLabel("--")
        self.img_info.setFont(QFont("Monospace", 9))
        self.img_info.setStyleSheet("color: #666; padding: 2px;")

        left_panel.addWidget(self.img_label, stretch=1)
        left_panel.addWidget(self.tag_status)
        left_panel.addWidget(self.img_info)

        # ── 右侧: 关节卡片 + 采集日志 ────────────────────────────────
        right_panel = QVBoxLayout()
        right_panel.setSpacing(8)

        title = QLabel("关节角度")
        title.setFont(QFont("Monospace", 14, QFont.Bold))
        title.setStyleSheet("color: #ddd;")
        title.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(title)

        self.cards = []
        card_grid = QGridLayout()
        card_grid.setSpacing(6)
        for i in range(NUM_JOINTS):
            c = JointCard(JOINT_NAMES[i], CARD_COLORS[i])
            self.cards.append(c)
            row, col = divmod(i, 2)
            card_grid.addWidget(c, row, col)
        right_panel.addLayout(card_grid)

        # 采集计数
        self.counter_label = QLabel("已采集: 0 组")
        self.counter_label.setFont(QFont("Monospace", 13, QFont.Bold))
        self.counter_label.setStyleSheet("color: #f39c12;")
        self.counter_label.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.counter_label)

        # 自动状态指示灯
        self.auto_status = QLabel("⏸ 等待 Tag...")
        self.auto_status.setFont(QFont("Monospace", 12))
        self.auto_status.setStyleSheet("color: #888; padding: 4px;")
        self.auto_status.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.auto_status)

        # 日志区
        log_label = QLabel("采集日志:")
        log_label.setFont(QFont("Monospace", 9))
        log_label.setStyleSheet("color: #888;")
        right_panel.addWidget(log_label)

        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFont(QFont("Monospace", 9))
        self.log_area.setMaximumHeight(160)
        self.log_area.setStyleSheet("""
            QTextEdit {
                background-color: #121220;
                color: #aaa;
                border: 1px solid #333;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        right_panel.addWidget(self.log_area)

        # 快捷键提示
        hint = QLabel("Q 退出 | 自动模式: 检测到 Tag + 关节变化 → 自动存")
        hint.setFont(QFont("Monospace", 8))
        hint.setStyleSheet("color: #555; padding: 2px;")
        hint.setAlignment(Qt.AlignCenter)
        hint.setWordWrap(True)
        right_panel.addWidget(hint)

        right_panel.addStretch()

        # ── 组合 ─────────────────────────────────────────────────────
        left_wrapper = QWidget()
        left_wrapper.setLayout(left_panel)
        right_wrapper = QWidget()
        right_wrapper.setLayout(right_panel)
        right_wrapper.setMaximumWidth(340)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_wrapper)
        splitter.addWidget(right_wrapper)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter)

        # ── 定时刷新 100ms ───────────────────────────────────────────
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

        self.setFocusPolicy(Qt.StrongFocus)

    # ── CSV 初始化 ────────────────────────────────────────────────────

    def _init_csv(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        if not os.path.isfile(self._csv_path):
            with open(self._csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "index", "timestamp",
                    "j1", "j2", "j3", "j4", "j5", "j6",
                    "tag_tx", "tag_ty", "tag_tz",
                    "tag_rx", "tag_ry", "tag_rz",
                ])

    # ── 键盘 ──────────────────────────────────────────────────────────

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_Q:
            self.close()
        else:
            super().keyPressEvent(event)

    # ── 日志 ──────────────────────────────────────────────────────────

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_area.append(f"[{ts}] {msg}")
        # 保留最近 100 行
        while self.log_area.document().blockCount() > 100:
            cursor = self.log_area.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    # ── 主循环 ────────────────────────────────────────────────────────

    def _tick(self):
        buf = self.buf

        # 1. 图像 + ArUco
        bgr = buf.get_image()
        tag_now = False
        if bgr is not None:
            K, D = buf.get_camera_params()
            corners, ids, target_rvec, target_tvec, dict_name = \
                self.detector.detect(bgr, K, D)

            display = bgr.copy()
            all_ids_str = "?"

            if ids is not None and len(ids) > 0:
                # ── 画所有检测到的 marker (不管是不是目标) ──
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                all_ids_str = str(ids.flatten().tolist())

                if target_rvec is not None:
                    # 目标 ID 找到了, 画坐标轴 + 位姿文字
                    cv2.drawFrameAxes(display, K, D, target_rvec, target_tvec,
                                      TAG_SIZE * 1.5, 2)
                    t = target_tvec.flatten()
                    cv2.putText(display,
                                f"ID={TAG_ID} t=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}]",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)

                    buf.put_tag_pose(target_rvec, target_tvec)
                    tag_now = True

                    self.tag_status.setText(
                        f"Tag ID={TAG_ID} ✓ | dict={dict_name} | all={all_ids_str} | "
                        f"t=[{t[0]:.3f} {t[1]:.3f} {t[2]:.3f}] m")
                    self.tag_status.setStyleSheet(
                        "color: #2ecc71; font-family: Monospace; font-size: 10px; "
                        "padding: 2px;")
                else:
                    # 检测到其他 marker 但不是目标 ID
                    buf.clear_tag()
                    self.tag_status.setText(
                        f"检测到 IDs={all_ids_str} (dict={dict_name}), "
                        f"但无目标 ID={TAG_ID}")
                    self.tag_status.setStyleSheet(
                        "color: #f39c12; font-family: Monospace; font-size: 10px; "
                        "padding: 2px;")
            else:
                buf.clear_tag()
                self.tag_status.setText(f"Tag: 未检测到 (dict={dict_name})")
                self.tag_status.setStyleSheet(
                    "color: #e74c3c; font-family: Monospace; font-size: 10px; "
                    "padding: 2px;")

            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            if pixmap and not pixmap.isNull():
                lbl_sz = self.img_label.size()
                if lbl_sz.width() > 10 and lbl_sz.height() > 10:
                    pixmap = pixmap.scaled(lbl_sz, Qt.KeepAspectRatio,
                                           Qt.SmoothTransformation)
                self.img_label.setPixmap(pixmap)

            self.img_info.setText(
                f"{w}x{h} | 帧 #{buf.frame_count} | "
                f"dict={dict_name} | "
                f"内参: {'OK' if buf.has_camera_info else 'default'}")

        # 2. 关节卡片
        joints = buf.get_joints()
        for i, card in enumerate(self.cards):
            card.update_value(joints[i])

        # 3. 自动采集 — Tag 检测到 + 关节有效 → 比对指纹 → 去重写入
        if tag_now:
            self._try_auto_save(joints)
        else:
            self._auto_status_text = "⏸ 等待 Tag..."

        # 更新状态指示灯
        self.auto_status.setText(self._auto_status_text)
        if "✓" in self._auto_status_text:
            self.auto_status.setStyleSheet("color: #2ecc71; font-family: Monospace; "
                                            "font-size: 12px; padding: 4px;")
        elif "已存" in self._auto_status_text or "跳过" in self._auto_status_text:
            self.auto_status.setStyleSheet("color: #f39c12; font-family: Monospace; "
                                            "font-size: 12px; padding: 4px;")
        else:
            self.auto_status.setStyleSheet("color: #888; font-family: Monospace; "
                                            "font-size: 12px; padding: 4px;")

    # ── 自动采集 (检测到即存, 0.5s 最小间隔防连拍) ──────────────────

    def _try_auto_save(self, joints):
        """Tag 检测到 → 同时取 tag 位姿 + 关节角度 → 写入 CSV"""
        now = time.time()

        # 0.5s 最小间隔 (防同一帧连拍)
        if now - self._last_save_time < 0.5:
            return

        rvec, tvec = self.buf.get_tag_pose()
        if rvec is None or tvec is None:
            return

        # ── 写入 CSV ─────────────────────────────────────────────
        self._save_counter += 1
        idx = self._save_counter
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
        tx, ty, tz = tvec.flatten()
        rx, ry, rz = rvec.flatten()

        with open(self._csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                idx, timestamp,
                joints[0], joints[1], joints[2],
                joints[3], joints[4], joints[5],
                tx, ty, tz, rx, ry, rz,
            ])

        self._last_save_time = now

        self.counter_label.setText(f"已采集: {idx} 组")
        self._auto_status_text = f"✓ 已存 #{idx}"

        # 终端输出
        print(f"\n{'='*60}")
        print(f"[采集 #{idx:03d}] {timestamp}")
        print(f"  Tag 位置 (camera_T_tag xyz):  [{tx:.4f}  {ty:.4f}  {tz:.4f}] m")
        print(f"  Tag 姿态 (Rodrigues rvec):    [{rx:.4f}  {ry:.4f}  {rz:.4f}]")
        print(f"  关节角度 (J1~J6 rad):")
        print(f"    {joints[0]:+.4f}  {joints[1]:+.4f}  {joints[2]:+.4f}  "
              f"{joints[3]:+.4f}  {joints[4]:+.4f}  {joints[5]:+.4f}")
        print(f"  关节角度 (J1~J6 deg):")
        print(f"    {joints[0]*57.3:+.1f}  {joints[1]*57.3:+.1f}  {joints[2]*57.3:+.1f}  "
              f"{joints[3]*57.3:+.1f}  {joints[4]*57.3:+.1f}  {joints[5]*57.3:+.1f}")
        print(f"{'='*60}")

        # 闪烁
        self.counter_label.setStyleSheet("color: #2ecc71; font-family: Monospace; "
                                          "font-size: 13px; font-weight: bold;")
        QTimer.singleShot(400, lambda: self.counter_label.setStyleSheet(
            "color: #f39c12; font-family: Monospace; font-size: 13px; font-weight: bold;"))


# ============================================================================
# main
# ============================================================================

CAMERA_IMAGE_TOPIC = "/camera/camera/color/image_raw"


def _camera_process_running() -> bool:
    """检测是否有 Realsense 相机驱动进程已在运行 (不关心话题是否已发布)。"""
    try:
        r = subprocess.run(["pgrep", "-f", "realsense2_camera_node"],
                           capture_output=True, text=True, timeout=3)
        return r.returncode == 0
    except Exception:
        return False


def ensure_camera_running(node: Node) -> bool:
    """
    如果相机话题没有数据, 则在后台启动 Realsense 相机驱动。
    返回 True 表示相机驱动已经在发布图像 (或本函数刚把它拉起来)。

    与 AR_bringup.launch.py 同时启动时, 相机进程可能已在初始化 (USB 尚未就绪,
    话题未发布)。此时必须检测"进程是否在运行"而非只看话题, 否则会重复启动
    相机导致 xioctl Device busy。
    """
    def _topic_exists(timeout: float = 6.0) -> bool:
        """用 ros2 topic list 探测话题是否存在 (输出即时, 无缓冲问题)"""
        try:
            r = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, text=True, timeout=timeout)
            return CAMERA_IMAGE_TOPIC in r.stdout
        except Exception:
            return False

    # 已有数据 → 不需要自己启动
    if _topic_exists():
        return True

    # 相机进程已在运行 (可能是 AR_bringup 或别的脚本起的) → 不要再重复启动
    if _camera_process_running():
        print("[相机] 检测到相机驱动已在运行 (可能是 AR_bringup 启动的), 等待就绪...")
        for _ in range(20):
            time.sleep(1)
            if _topic_exists():
                print("[相机] 相机驱动已就绪, 开始接收图像")
                return True
        print("[相机] 相机进程在运行但迟迟无话题, 请检查相机连接")
        return False

    # 未启动 → 拉起来
    if not shutil.which("ros2"):
        print("[相机] 找不到 ros2 命令, 请先 source 环境 (source /opt/ros/humble/setup.bash)")
        return False
    rs_launch = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
        "launch_camera.sh")
    if not os.path.isfile(rs_launch):
        print("[相机] 未找到 launch_camera.sh, 请手动启动: "
              "ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera")
        return False

    print("[相机] 检测到相机未运行, 自动启动 Realsense 驱动...")
    try:
        subprocess.Popen(
            [rs_launch],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
    except Exception as e:
        print(f"[相机] 启动失败: {e}")
        return False

    # 等待相机就绪 (最多 20s)
    for i in range(20):
        time.sleep(1)
        if _topic_exists():
            print("[相机] 相机驱动已就绪, 开始接收图像")
            return True
    print("[相机] 相机驱动启动中..., 若长时间无图像请检查相机连接")
    return False


def _spin_until_shutdown(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    rclpy.init()
    buf = DataBuffer()
    node = CalibSubscriber(buf)
    det = ArucoDetector()

    spin_thread = threading.Thread(target=_spin_until_shutdown, args=(node,), daemon=True)
    spin_thread.start()

    # ── 确保相机驱动在运行 (不在则自动启动) ──
    cam_ok = ensure_camera_running(node)
    if not cam_ok:
        print("[提示] 相机可能未就绪, 图像区域将显示'等待图像...'。"
              "可手动启动: ros2 launch realsense2_camera rs_launch.py "
              "camera_name:=camera camera_namespace:=camera")

    app = QApplication(sys.argv)

    import signal as _signal
    _signal.signal(_signal.SIGINT, lambda *_: app.quit())

    win = MainWindow(buf, det)
    win.show()

    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        exit_code = 0

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
