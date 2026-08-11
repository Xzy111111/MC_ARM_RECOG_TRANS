#!/usr/bin/env python3
"""ArUco ID=12 检测显示 + TF发布 (PyQt5 + OpenCV + ROS2)"""
import sys, threading, signal, math, os, subprocess, shutil, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2, numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QImage, QFont, QKeyEvent

TARGET_ID = 12
TAG_SIZE = 0.128
PARENT_FRAME = "camera_color_optical_frame"
CHILD_FRAME = "camera_marker"

# D435i 相机发布 RELIABLE 图像, 订阅必须也用 RELIABLE 才能收到数据
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
IMG_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    params = cv2.aruco.DetectorParameters()
except:
    params = cv2.aruco.DetectorParameters_create()
try:
    detector = cv2.aruco.ArucoDetector(dictionary, params)
except:
    detector = None


def rot_to_quat(R):
    """3x3旋转矩阵 -> (x,y,z,w)"""
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


class Node_(Node):
    def __init__(self, img_cb, info_cb):
        super().__init__("aruco_viewer")
        self.sub = self.create_subscription(Image, "/camera/camera/color/image_raw", img_cb, IMG_QOS)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", info_cb, 1)
        self.tf_br = TransformBroadcaster(self)
        print("已订阅 image_raw + camera_info, TF广播器就绪")


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


class Win(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ArUco Viewer ID=12 + TF")
        self.setMinimumSize(640, 480)
        self.setStyleSheet("background:#1e1e2e")
        l = QVBoxLayout(self)
        l.setContentsMargins(0,0,0,0)
        self.img = QLabel("等待图像...")
        self.img.setAlignment(Qt.AlignCenter)
        self.img.setStyleSheet("color:#888;font-size:18px")
        self.img.setScaledContents(False)
        l.addWidget(self.img)
        self.st = QLabel("等待 ROS...")
        self.st.setFont(QFont("Monospace",10))
        self.st.setStyleSheet("color:gray;padding:4px")
        self.st.setAlignment(Qt.AlignCenter)
        l.addWidget(self.st)
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.lock = threading.Lock()
        self.frame = None
        self._cam = None
        self._dist = None

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q: self.close()

    def on_img(self, msg):
        with self.lock:
            self.frame = to_bgr(msg)

    def on_cam_info(self, msg):
        with self.lock:
            self._cam = np.array(msg.k, dtype=np.float64).reshape(3,3)
            self._dist = np.array(msg.d, dtype=np.float64)

    def tick(self):
        with self.lock:
            f = self.frame
            cam = self._cam
            dist = self._dist
            self.frame = None
        if f is None: return

        ok = False
        try:
            if detector: corners, ids, _ = detector.detectMarkers(f)
            else: corners, ids, _ = cv2.aruco.detectMarkers(f, dictionary, parameters=params)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(f, corners, ids)
                for i, mid in enumerate(ids.flatten()):
                    if mid == TARGET_ID:
                        ok = True
                        c = corners[i][0].mean(axis=0).astype(int)
                        cv2.putText(f, "ID=12", (c[0]-30,c[1]-15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                        if cam is not None:
                            try:
                                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                                    corners[i], TAG_SIZE, cam, dist)
                                cv2.drawFrameAxes(f, cam, dist, rvec[0], tvec[0], 0.05)
                                # 发布 TF
                                rmat, _ = cv2.Rodrigues(rvec[0])
                                q = rot_to_quat(rmat)
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
                                self.node.tf_br.sendTransform(t)
                                print(f"TF: {PARENT_FRAME} -> {CHILD_FRAME} "
                                      f"t=({tvec[0][0][0]:.3f},{tvec[0][0][1]:.3f},{tvec[0][0][2]:.3f})")
                            except Exception:
                                pass
        except Exception:
            pass

        pix = QPixmap.fromImage(QImage(
            cv2.cvtColor(f, cv2.COLOR_BGR2RGB).data,
            f.shape[1], f.shape[0], 3*f.shape[1], QImage.Format_RGB888))
        if pix:
            s = self.img.size()
            if s.width()>10 and s.height()>10:
                pix = pix.scaled(s, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.img.setPixmap(pix)
        self.st.setText("TF发布中 ✓" if ok else "未检测到")


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
    n = Node_(None, None)
    w = Win(n)
    n.sub.destroy()
    n.sub = n.create_subscription(Image, "/camera/camera/color/image_raw", w.on_img,
                                  IMG_QOS)
    n.info_sub.destroy()
    n.info_sub = n.create_subscription(CameraInfo, "/camera/camera/color/camera_info", w.on_cam_info, 1)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    w.show()
    r = app.exec_()
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(r)

if __name__ == "__main__":
    main()
