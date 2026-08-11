#!/usr/bin/env python3
"""
ArUco ID=12 检测 + 发布 TF: camera_color_optical_frame → camera_marker
订阅 /camera/camera/color/image_raw + /camera/camera/color/camera_info
"""
import sys, threading, signal, os, subprocess, shutil, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
import math

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QImage, QFont, QKeyEvent

# ---- 检测参数 ----
TARGET_ID = 12
MARKER_SIZE = 0.128          # 米 (来自标定注释)
DICT_NAME = "DICT_ARUCO_ORIGINAL"
PARENT_FRAME = "camera_color_optical_frame"
CHILD_FRAME = "camera_marker"

# D435i 相机发布 RELIABLE 图像, 订阅必须也用 RELIABLE 才能收到数据
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
IMG_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

# ---- OpenCV 初始化 ----
aruco_dict = cv2.aruco.getPredefinedDictionary(
    getattr(cv2.aruco, DICT_NAME)
)
try:
    params = cv2.aruco.DetectorParameters()
except AttributeError:
    params = cv2.aruco.DetectorParameters_create()
try:
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
except AttributeError:
    detector = None

print(f"=== ArUco TF Publisher: ID={TARGET_ID} size={MARKER_SIZE}m ===")
print(f"词典: {DICT_NAME}")
print(f"TF: {PARENT_FRAME} → {CHILD_FRAME}")


# ---- ROS node ----
class ArucoNode(Node):
    def __init__(self, img_cb, info_cb):
        super().__init__("aruco_tf_publisher")
        self.sub_img = self.create_subscription(
            Image, "/camera/camera/color/image_raw", img_cb, IMG_QOS)
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", info_cb, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        print("已订阅 image_raw + camera_info，TF 广播器就绪")


# ---- 工具函数 ----
def _rot_to_quat(R):
    """3x3旋转矩阵 -> (qx, qy, qz, qw)"""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        return ((R[2,1]-R[1,2])/S, (R[0,2]-R[2,0])/S, (R[1,0]-R[0,1])/S, 0.25*S)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        return (0.25*S, (R[0,1]+R[1,0])/S, (R[0,2]+R[2,0])/S, (R[2,1]-R[1,2])/S)
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        return ((R[0,1]+R[1,0])/S, 0.25*S, (R[1,2]+R[2,1])/S, (R[0,2]-R[2,0])/S)
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        return ((R[0,2]+R[2,0])/S, (R[1,2]+R[2,1])/S, 0.25*S, (R[1,0]-R[0,1])/S)


# ---- 图像转换 ----
def to_bgr(msg):
    if msg.encoding == "rgb8":
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == "bgr8":
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
    elif msg.encoding in ("mono8", "8UC1"):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width).copy()
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif msg.encoding == "bgra":
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4).copy()
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif msg.encoding in ("16UC1", "mono16"):
        img = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
        img = (img >> 8).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()


# ---- 主窗口 ----
class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle(f"ArUco ID={TARGET_ID}  TF Publisher")
        self.setMinimumSize(640, 480)
        self.setStyleSheet("background:#1e1e2e")
        l = QVBoxLayout(self)
        l.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel("等待图像...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("color:#888;font-size:18px")
        self.label.setScaledContents(False)
        l.addWidget(self.label)
        self.status = QLabel("等待 ROS...")
        self.status.setFont(QFont("Monospace", 10))
        self.status.setStyleSheet("color:gray;padding:4px")
        self.status.setAlignment(Qt.AlignCenter)
        l.addWidget(self.status)

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        self.lock = threading.Lock()
        self.frame = None
        self.cam_mat = None
        self.dist = None

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            self.close()

    def on_img(self, msg):
        with self.lock:
            self.frame = to_bgr(msg)

    def on_cam_info(self, msg):
        with self.lock:
            self.cam_mat = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(msg.d, dtype=np.float64)

    def tick(self):
        with self.lock:
            f = self.frame
            cam = self.cam_mat
            dist = self.dist
            self.frame = None
        if f is None:
            return

        found = False
        try:
            if detector:
                corners, ids, _ = detector.detectMarkers(f)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(f, aruco_dict, parameters=params)
        except Exception:
            corners, ids = None, None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(f, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                if mid == TARGET_ID:
                    found = True
                    c = corners[i][0].mean(axis=0).astype(int)
                    cv2.putText(f, f"ID={TARGET_ID}", (c[0]-30, c[1]-15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                    # ---- 发布 TF ----
                    if cam is not None:
                        try:
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                                corners[i], MARKER_SIZE, cam, dist)
                            rmat, _ = cv2.Rodrigues(rvec[0])
                            # 旋转矩阵 → 四元数
                            q = _rot_to_quat(rmat)
                        except Exception:
                            q = (0, 0, 0, 1)
                            tvec = np.zeros((1, 1, 3))
                            tvec[0][0][2] = 0.0

                        t = TransformStamped()
                        t.header.stamp = self.node.get_clock().now().to_msg()
                        t.header.frame_id = PARENT_FRAME
                        t.child_frame_id = CHILD_FRAME
                        t.transform.translation.x = float(tvec[0][0][0])
                        t.transform.translation.y = float(tvec[0][0][1])
                        t.transform.translation.z = float(tvec[0][0][2])
                        t.transform.rotation.x = q[0]
                        t.transform.rotation.y = q[1]
                        t.transform.rotation.z = q[2]
                        t.transform.rotation.w = q[3]
                        self.node.tf_broadcaster.sendTransform(t)

                        print(f"\n===== 检测到 ID={TARGET_ID} 发布 TF =====")
                        print(f"位置: x={tvec[0][0][0]:.3f}  y={tvec[0][0][1]:.3f}  z={tvec[0][0][2]:.3f}")
                        print(f"帧: {PARENT_FRAME} → {CHILD_FRAME}")
                        print("=====================================")

        pix = QPixmap.fromImage(QImage(
            cv2.cvtColor(f, cv2.COLOR_BGR2RGB).data,
            f.shape[1], f.shape[0], 3 * f.shape[1], QImage.Format_RGB888))
        if pix:
            s = self.label.size()
            if s.width() > 10 and s.height() > 10:
                pix = pix.scaled(s, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.label.setPixmap(pix)
        self.status.setText("TF 发布中 ✓" if found else "未检测到")


# ---- main ----

CAMERA_IMAGE_TOPIC = "/camera/camera/color/image_raw"
_CAMERA_LAUNCHED = None      # 全局记住后台相机进程, 避免重复启动


def camera_process_running() -> bool:
    """检测是否有 Realsense 相机驱动进程已在运行 (不关心话题是否已发布)。"""
    try:
        r = subprocess.run(["pgrep", "-f", "realsense2_camera_node"],
                           capture_output=True, text=True, timeout=3)
        return r.returncode == 0
    except Exception:
        return False


def ensure_camera_running() -> bool:
    """相机话题无数据时, 自动后台启动 Realsense 驱动。返回相机是否就绪。

    与 AR_bringup.launch.py 同时启动时, 相机进程可能已在初始化 (USB 尚未就绪,
    话题未发布)。此时必须检测"进程是否在运行"而非只看话题, 否则会重复启动
    相机导致 xioctl Device busy。
    """
    global _CAMERA_LAUNCHED

    def topic_exists(timeout=6.0):
        """用 ros2 topic list 探测话题是否存在 (输出即时, 无缓冲问题)"""
        try:
            r = subprocess.run(["ros2", "topic", "list"],
                               capture_output=True, text=True, timeout=timeout)
            return CAMERA_IMAGE_TOPIC in r.stdout
        except Exception:
            return False

    # 情形 1: 话题已发布 → 相机就绪, 直接用
    if topic_exists():
        return True

    # 情形 2: 相机进程已在运行 (可能是 AR_bringup 或别的脚本起的) →
    #         不要再重复启动, 等它出话题即可
    if camera_process_running():
        print("[相机] 检测到相机驱动已在运行 (可能是 AR_bringup 启动的), 等待就绪...")
        for _ in range(20):
            time.sleep(1)
            if topic_exists():
                print("[相机] 相机驱动已就绪, 开始接收图像")
                return True
        print("[相机] 相机进程在运行但迟迟无话题, 请检查相机连接")
        return False

    # 情形 3: 本脚本自己启动过 → 等它出话题
    if _CAMERA_LAUNCHED is not None and _CAMERA_LAUNCHED.poll() is None:
        for _ in range(20):
            time.sleep(1)
            if topic_exists():
                print("[相机] 相机驱动已就绪")
                return True
        return False

    if not shutil.which("ros2"):
        print("[相机] 找不到 ros2, 请先 source /opt/ros/humble/setup.bash")
        return False
    rs_launch = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "..", "..", "launch_camera.sh")
    if not os.path.isfile(rs_launch):
        print("[相机] 未找到 launch_camera.sh, 请手动启动 Realsense 驱动")
        return False

    print("[相机] 检测到相机未运行, 自动启动 Realsense 驱动...")
    try:
        _CAMERA_LAUNCHED = subprocess.Popen(
            [rs_launch], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
    except Exception as e:
        print(f"[相机] 启动失败: {e}")
        return False

    for _ in range(20):
        time.sleep(1)
        if topic_exists():
            print("[相机] 相机驱动已就绪, 开始接收图像")
            return True
    print("[相机] 相机驱动启动中..., 若长时间无图像请检查相机连接")
    return False


def main():
    ensure_camera_running()

    rclpy.init()
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    node = ArucoNode(None, None)
    win = MainWindow(node)
    # 重接回调
    node.sub_img.destroy()
    node.sub_img = node.create_subscription(
        Image, "/camera/camera/color/image_raw", win.on_img, IMG_QOS)
    node.sub_info.destroy()
    node.sub_info = node.create_subscription(
        CameraInfo, "/camera/camera/color/camera_info", win.on_cam_info, 1)

    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    win.show()
    ret = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
