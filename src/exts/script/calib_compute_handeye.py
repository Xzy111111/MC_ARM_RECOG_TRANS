#!/usr/bin/env python3
"""
手眼标定计算脚本 (Eye-on-Hand AX=XB)
读取 calib_data.csv，正运动学 + OpenCV calibrateHandEye 求解 link6_T_camera

用法:
   python3 calib_compute_handeye.py                    # 使用最新的 calib_data.csv
   python3 calib_compute_handeye.py <csv_path>         # 指定 CSV 文件
   python3 calib_compute_handeye.py --list             # 列出所有 CSV 文件

输出:
  - 手眼矩阵 link6_T_camera (4×4 齐次)
  - 可用于 static_transform_publisher 的命令行参数
"""

import sys
import os
import csv
import math
import glob

import numpy as np

# OpenCV 可能为 cv2 或 cv2.so
try:
    import cv2
except ImportError:
    print("[ERROR] 请安装 opencv-python: pip install opencv-python")
    sys.exit(1)


# ============================================================================
# DATA_DIR
# ============================================================================

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


# ============================================================================
# Piper 6 轴正运动学 (从 car_with_piper.urdf.xacro 提取)
# ============================================================================

def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0],
                     [0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _transl(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    """roll-pitch-yaw (fixed axis XYZ) → 4×4 homogeneous"""
    return _rot_z(y) @ _rot_y(p) @ _rot_x(r)


def _origin_xyz_rpy(x, y, z, r, p, yaw):
    """URDF joint origin: translation then rotation"""
    return _transl(x, y, z) @ _rpy(r, p, yaw)


# URDF joint origins (link_i-1 → link_i 的静态变换)
# 格式: (x, y, z, roll, pitch, yaw)
JOINT_ORIGINS = [
    (0.0,       0.0,       0.123,   0.0,         0.0,         0.0),          # joint1: base_link→link1
    (0.0,       0.0,       0.0,     1.5707963,  -0.1357866,  -3.1415926),    # joint2: link1→link2
    (0.28503,   0.0,       0.0,     0.0,         0.0,        -1.7938494),    # joint3: link2→link3
    (-0.02198, -0.25075,   0.0,     1.5707963,   0.0,         0.0),          # joint4: link3→link4
    (0.0,       0.0,       0.0,    -1.5707963,   0.0,         0.0),          # joint5: link4→link5
    (8.8259e-05, -0.091,   0.0,     1.5707963,   0.0,         0.0),          # joint6: link5→link6
]


def forward_kinematics(q: list) -> np.ndarray:
    """
    Piper 正运动学: 6 个关节角度 → base_T_link6 (4×4)

    参数:
        q: [q1, q2, q3, q4, q5, q6] (弧度)
    返回:
        base_T_link6: 4×4 np.array (齐次变换矩阵)

    公式: T = Π_i (T_origin_i · Rz(q_i))
    """
    T = np.eye(4, dtype=np.float64)
    for i, qi in enumerate(q):
        x, y, z, r, p, yaw = JOINT_ORIGINS[i]
        T_origin = _origin_xyz_rpy(x, y, z, r, p, yaw)
        T_rot = _rot_z(qi)
        T = T @ T_origin @ T_rot
    return T


def rotation_matrix_to_rpy(R: np.ndarray) -> tuple:
    """3×3 旋转矩阵 → (roll, pitch, yaw) 弧度"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


# ============================================================================
# CSV 读取
# ============================================================================

def find_latest_csv(data_dir: str = DATA_DIR) -> str:
    """返回 data/ 目录下最新的 calib_data.csv，不存在则报错退出"""
    csv_path = os.path.join(data_dir, "calib_data.csv")
    if os.path.isfile(csv_path):
        return csv_path

    # 搜索所有 CSV
    files = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if files:
        return files[-1]

    print(f"[ERROR] 在 {data_dir} 下找不到任何 CSV 文件")
    sys.exit(1)


def list_csv_files(data_dir: str = DATA_DIR):
    """列出所有 CSV 文件"""
    files = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if not files:
        print(f"  (空 — {data_dir}/ 下没有 CSV 文件)")
        return
    for f in files:
        sz = os.path.getsize(f)
        print(f"  {os.path.basename(f):40s}  {sz:>8d} bytes")


def read_calib_csv(csv_path: str) -> list:
    """
    读取 CSV，返回数据行列表。

    每行 dict:
        index, timestamp,
        j1..j6 (float),
        tag_tx, tag_ty, tag_tz (float),
        tag_rx, tag_ry, tag_rz (float)
    """
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                data = {
                    "index": int(row["index"]),
                    "timestamp": row.get("timestamp", ""),
                    "joints": [
                        float(row["j1"]), float(row["j2"]), float(row["j3"]),
                        float(row["j4"]), float(row["j5"]), float(row["j6"]),
                    ],
                }
                # tag 位姿 (可能为空字符串)
                try:
                    data["tag_tvec"] = np.array([
                        float(row["tag_tx"]), float(row["tag_ty"]), float(row["tag_tz"])
                    ], dtype=np.float64)
                    data["tag_rvec"] = np.array([
                        float(row["tag_rx"]), float(row["tag_ry"]), float(row["tag_rz"])
                    ], dtype=np.float64)
                    data["has_tag"] = True
                except (ValueError, KeyError):
                    data["tag_tvec"] = None
                    data["tag_rvec"] = None
                    data["has_tag"] = False

                rows.append(data)
            except (ValueError, KeyError) as e:
                print(f"  [WARN] 跳过无效行: {e}")
                continue

    return rows


# ============================================================================
# 手眼标定求解
# ============================================================================

def rodrigues_to_rmat(rvec: np.ndarray) -> np.ndarray:
    """Rodrigues 旋转向量 → 3×3 旋转矩阵"""
    rmat, _ = cv2.Rodrigues(rvec)
    return rmat


def solve_handeye(rows: list, method: int = None) -> dict:
    """
    求解 AX=XB 手眼标定。

    eye-on-hand 模式:
      base_T_camera = base_T_link6 · link6_T_camera   (1)
      base_T_camera = base_T_tag · tag_T_camera        (2)

    求解 X = link6_T_camera，使得 A·X = X·B

    OpenCV 接口:
      calibrateHandEye(R_gripper2base, t_gripper2base,
                       R_target2cam,  t_target2cam) → R_cam2gripper, t_cam2gripper

    返回 link6_T_camera = [R_cam2gripper | t_cam2gripper]
    """
    if method is None:
        method = cv2.CALIB_HAND_EYE_TSAI

    method_names = {
        cv2.CALIB_HAND_EYE_TSAI: "Tsai-Lenz",
        cv2.CALIB_HAND_EYE_PARK: "Park-Martin",
        cv2.CALIB_HAND_EYE_HORAUD: "Horaud",
        cv2.CALIB_HAND_EYE_ANDREFF: "Andreff",
        cv2.CALIB_HAND_EYE_DANIILIDIS: "Daniilidis",
    }

    # 过滤有 tag 位姿的行
    valid = [r for r in rows if r["has_tag"]]
    if len(valid) < 3:
        print(f"[ERROR] 有效数据不足: {len(valid)} 行 (至少需要 3 组)")
        sys.exit(1)

    print(f"\n使用 {len(valid)} 组有效数据 (共 {len(rows)} 行 CSV)")

    R_gripper2base = []  # base_T_link6 的旋转部分
    t_gripper2base = []  # base_T_link6 的平移部分
    R_target2cam = []    # camera_T_tag 的旋转部分 (tag在相机坐标系)
    t_target2cam = []    # camera_T_tag 的平移部分

    for r in valid:
        # 正运动学
        T_base_link6 = forward_kinematics(r["joints"])
        R_gripper2base.append(T_base_link6[:3, :3].copy())
        t_gripper2base.append(T_base_link6[:3, 3].copy())

        # tag 位姿 (camera_T_tag)
        R_tag_cam = rodrigues_to_rmat(r["tag_rvec"])
        t_tag_cam = r["tag_tvec"].reshape(3)
        R_target2cam.append(R_tag_cam)
        t_target2cam.append(t_tag_cam)

    print(f"\n{'='*70}")
    print(f"  手眼标定结果 (Eye-on-Hand)")
    print(f"{'='*70}")

    # 多头求解
    results = {}
    for m in [cv2.CALIB_HAND_EYE_TSAI,
              cv2.CALIB_HAND_EYE_PARK,
              cv2.CALIB_HAND_EYE_HORAUD,
              cv2.CALIB_HAND_EYE_ANDREFF,
              cv2.CALIB_HAND_EYE_DANIILIDIS]:
        try:
            R_cam2grip, t_cam2grip = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=m)
            results[m] = (R_cam2grip, t_cam2grip.flatten())
        except cv2.error as e:
            print(f"  {method_names.get(m, m):20s}: FAILED ({e})")

    if not results:
        print("[ERROR] 所有算法均失败，检查输入数据")
        sys.exit(1)

    # 用 Tsai 作为主结果
    if method in results:
        R, t = results[method]
    else:
        # fallback 到第一个成功的方法
        m0 = list(results.keys())[0]
        R, t = results[m0]
        method = m0

    print(f"\n  主算法: {method_names[method]}")
    print(f"\n  link6_T_camera (4×4 齐次矩阵):")
    T_handeye = np.eye(4)
    T_handeye[:3, :3] = R
    T_handeye[:3, 3] = t
    for row_line in T_handeye:
        print(f"    [{row_line[0]:10.6f}  {row_line[1]:10.6f}  "
              f"{row_line[2]:10.6f}  {row_line[3]:10.6f}]")

    r, p, yaw = rotation_matrix_to_rpy(R)
    print(f"\n  平移 (x, y, z):  [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}] m")
    print(f"  旋转 (r, p, y):  [{r:.6f}, {p:.6f}, {yaw:.6f}] rad")
    print(f"                 = [{math.degrees(r):.4f}°, {math.degrees(p):.4f}°, "
          f"{math.degrees(yaw):.4f}°]")

    # 与其他方法对比
    print(f"\n  多算法对比:")
    for m_name, (R2, t2) in results.items():
        dR = np.linalg.norm(R2 - R, 'fro')
        dt = np.linalg.norm(t2 - t)
        marker = " ◄" if m_name == method else ""
        print(f"    {method_names.get(m_name, m_name):20s}  ΔR={dR:.6f}  Δt={dt:.6f} m{marker}")

    # 用于 AR_bringup.launch.py static TF 的命令行
    print(f"\n  {'─'*70}")
    print(f"  用于 AR_bringup.launch.py 的参数 (link6 → camera_link):")
    print(f"    --x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} \\")
    print(f"    --roll {r:.6f} --pitch {p:.6f} --yaw {yaw:.6f}")

    # ── 保存结果到文件 ──────────────────────────────────────────
    result_path = os.path.join(DATA_DIR, "calib_result.txt")
    with open(result_path, "w") as f:
        f.write(f"# 手眼标定结果 (Eye-on-Hand) — {method_names[method]}\n")
        f.write(f"# link6_T_camera 4×4 齐次矩阵:\n")
        for row_line in T_handeye:
            f.write(f"#   [{row_line[0]:10.6f}  {row_line[1]:10.6f}  "
                    f"{row_line[2]:10.6f}  {row_line[3]:10.6f}]\n")
        f.write(f"#\n")
        f.write(f"# static_transform_publisher 参数 (link6 → camera_link):\n")
        f.write(f"--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} \\\n")
        f.write(f"--roll {r:.6f} --pitch {p:.6f} --yaw {yaw:.6f}\n")
    print(f"\n  结果已保存到: {result_path}")

    return {
        "method": method_names[method],
        "T_handeye": T_handeye,
        "all_results": results,
    }


# ============================================================================
# 数据预览
# ============================================================================

def print_preview(rows: list):
    """打印 CSV 数据摘要"""
    print(f"\n  CSV 数据预览 ({len(rows)} 行):")
    print(f"  {'─'*75}")
    print(f"  {'#':>3s}  {'j1':>8s}  {'j2':>8s}  {'j3':>8s}  {'j4':>8s}  "
          f"{'j5':>8s}  {'j6':>8s}  {'tag':>5s}")
    print(f"  {'─'*75}")
    for r in rows:
        q = r["joints"]
        tag = "✓" if r["has_tag"] else "✗"
        print(f"  {r['index']:3d}  {q[0]:+8.4f}  {q[1]:+8.4f}  {q[2]:+8.4f}  "
              f"{q[3]:+8.4f}  {q[4]:+8.4f}  {q[5]:+8.4f}  {tag:>5s}")

    valid = [r for r in rows if r["has_tag"]]
    print(f"\n  有效数据 (含 tag 位姿): {len(valid)} / {len(rows)}")


# ============================================================================
# main
# ============================================================================

def main():
    csv_path = None

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "--list":
            print(f"CSV 文件列表 ({DATA_DIR}/):\n")
            list_csv_files(DATA_DIR)
            return
        elif arg == "--help" or arg == "-h":
            print(__doc__)
            return
        else:
            csv_path = arg

    if csv_path is None:
        csv_path = find_latest_csv(DATA_DIR)

    if not os.path.isfile(csv_path):
        print(f"[ERROR] 文件不存在: {csv_path}")
        sys.exit(1)

    print(f"读取: {csv_path}")
    rows = read_calib_csv(csv_path)

    if not rows:
        print("[ERROR] CSV 中没有有效数据行")
        sys.exit(1)

    print_preview(rows)
    solve_handeye(rows)


if __name__ == "__main__":
    main()
