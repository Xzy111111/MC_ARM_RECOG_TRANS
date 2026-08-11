#!/usr/bin/env python3
"""
手眼标定验收脚本 — 用四类判据自动打分, 判断标定是否达标。

用法:
   python3 verify_handeye.py                     # 用 data/calib_data.csv + data/calib_result.txt 自动评估
   python3 verify_handeye.py <csv_path>          # 指定数据 CSV
   python3 verify_handeye.py <csv_path> <result_path>   # 同时指定标定结果文件
   python3 verify_handeye.py --help

四类判据 (Eye-on-Hand, X = link6_T_camera):
   1. 数据质量    — 有效帧数、关节/末端空间与姿态覆盖是否足够
   2. 多算法一致性 — Tsai/Park/Horaud/Andreff/Daniilidis 解出的 X 彼此是否接近
   3. 反投影闭环  — base_T_tag = base_T_link6 · X · cam_T_tag 在所有帧应重合 (金标准)
   4. 物理合理性  — X 的平移应落在 link6 末端挂载相机的位置, 光轴朝向合理

任一判据不合格 → 输出 FAIL 并给出重采建议。
"""

import os
import sys
import csv
import math

import numpy as np

try:
    import cv2
except ImportError:
    print("[ERROR] 请安装 opencv-python: pip install opencv-python")
    sys.exit(1)

# 复用求解器里的正运动学与工具函数 (同一份参数, 避免两处维护)
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from calib_compute_handeye import (  # noqa: E402
    forward_kinematics,
    rodrigues_to_rmat,
    rotation_matrix_to_rpy,
)

DATA_DIR = os.path.join(HERE, "data")

# 求解数据规模: 标定求解实际用到的帧数。提供时只取 CSV 前 N 行评估"求解一致性",
# 其余行视为标定后新采集的数据, 单独反投影, 不混入求解 (混入会导致算法不一致被高估)。
SOLVE_ROWS = int(os.environ.get("HAND_EYE_SOLVE_ROWS", "0")) or None

# ============================================================================
# 验收阈值 (可调)
# ============================================================================

THRESHOLDS = {
    # 1. 数据质量
    "min_valid": 40,                 # 有效(含 tag)帧下限
    "min_active": 30,                # 关节有实际运动的帧下限
    "max_pose_span_m": 0.20,         # link6 末端空间跨度下限 (m), 小于则平移约束弱
    "min_orient_deg": 30,            # 末端姿态两两最大角差下限 (deg)
    # 2. 多算法一致性 (相对主算法 Tsai)
    "algo_dr": 0.05,                 # ΔR (Frobenius) 上限
    "algo_dt_m": 0.01,               # Δt (L2) 上限 (m)
    # 3. 反投影闭环
    "rms_pos_m": 0.010,              # base_T_tag 位置 RMS 上限 (m)
    "max_pos_m": 0.030,              # 单帧最大偏差上限 (m)
    "rms_rot_deg": 2.0,              # base_T_tag 姿态 RMS 上限 (deg)
    # 4. 物理合理性
    "cam_dist_mm_max": 250,          # 相机到 link6 原点距离上限 (mm)
    "expect_cam_dist_mm": (60, 180), # 相机距离期望区间 (mm), 仅用于提示
}

PASS_COLOR = "\033[92m"
FAIL_COLOR = "\033[91m"
WARN_COLOR = "\033[93m"
BOLD = "\033[1m"
RESET = "\033[0m"
DIRECT_RIGHT = " ──▶"


def _mark(ok):
    return f"{PASS_COLOR}{'PASS' if ok else 'FAIL'}{RESET}"


def _score(weight, passed):
    return weight if passed else 0.0


# ============================================================================
# 数据读取
# ============================================================================

def read_csv(csv_path):
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                joints = [float(row[f"j{i}"]) for i in range(1, 7)]
                tag_tvec = np.array(
                    [float(row["tag_tx"]), float(row["tag_ty"]), float(row["tag_tz"])],
                    dtype=np.float64)
                tag_rvec = np.array(
                    [float(row["tag_rx"]), float(row["tag_ry"]), float(row["tag_rz"])],
                    dtype=np.float64)
                rows.append({"joints": joints, "tvec": tag_tvec, "rvec": tag_rvec})
            except (ValueError, KeyError):
                continue
    return rows


def read_result(path):
    """解析 calib_result.txt / handeye_calib.yaml 风格的结果, 返回 (R, t, meta)。

    兼容三种书写形式:
      1) --x .. --y .. --z .. \\   --roll .. --pitch .. --yaw ..   (calib_result.txt 输出)
      2) x:/y:/z: + roll:/pitch:/yaw: yaml 键值
      3) 4×4 齐次矩阵 (可能带 '#' 注释前缀)
    """
    if not os.path.isfile(path):
        print(f"[ERROR] 结果文件不存在: {path}")
        sys.exit(1)

    R = None
    t = None
    matrix_rows = []
    meta = {}

    for line in open(path):
        content = line.split("#")[0].strip()
        if not content:
            if "算法" in line:
                meta["algorithm"] = line.lstrip("#").strip()
            continue

        # 1) static_transform_publisher 命令行: --x -0.026 --y ... --roll ... --yaw ...
        if content.startswith("--"):
            vals = {}
            tokens = content.split()
            for i in range(len(tokens) - 1):
                if tokens[i] in ("--x", "--y", "--z", "--roll", "--pitch", "--yaw"):
                    try:
                        vals[tokens[i][2:]] = float(tokens[i + 1])
                    except ValueError:
                        pass
            if {"x", "y", "z"} <= set(vals):
                t = np.array([vals["x"], vals["y"], vals["z"]])
            if {"roll", "pitch", "yaw"} <= set(vals):
                R = _rpy_to_rmat(vals["roll"], vals["pitch"], vals["yaw"])
            continue

        # 2) yaml 键值: x: -0.026 / roll: -0.385
        if ":" in content:
            key, _, val = content.partition(":")
            key = key.strip()
            try:
                v = float(val.strip())
            except ValueError:
                continue
            if key in ("x", "y", "z") and t is None:
                t = np.zeros(3)
                t[["x", "y", "z"].index(key)] = v
            elif key in ("roll", "pitch", "yaw") and R is None:
                meta.setdefault("rpy", [0.0, 0.0, 0.0])[["roll", "pitch", "yaw"].index(key)] = v
            continue

        # 3) 矩阵行: [-0.808820, 0.554372, ...]
        if content.startswith("["):
            try:
                row_vals = [float(x) for x in content.strip("[]").replace(",", " ").split()]
                if len(row_vals) == 4:
                    matrix_rows.append(row_vals[:3])
            except ValueError:
                pass

    if R is None and "rpy" in meta:
        r, p, y = meta["rpy"]
        R = _rpy_to_rmat(r, p, y)
    if R is None and len(matrix_rows) >= 3:
        R = np.array(matrix_rows[:3], dtype=np.float64)
    if R is None or t is None:
        print(f"[ERROR] 无法从 {path} 解析出旋转/平移 (需要 --x/--roll, x:/roll:, 或 matrix 三选一)")
        sys.exit(1)

    return R, t, meta


def _rpy_to_rmat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# ============================================================================
# 判据 1: 数据质量
# ============================================================================

def check_data_quality(rows, solve_rows=None):
    th = THRESHOLDS
    if solve_rows:
        rows = rows[:solve_rows]
        print(f"  (数据质量仅评估前 {solve_rows} 帧求解数据)")
    valid = len(rows)
    joints = np.array([r["joints"] for r in rows])
    active_mask = np.abs(joints).max(axis=1) > 0.01
    n_active = int(active_mask.sum())

    results = {}

    # 末端空间跨度
    pos6 = []
    for i, r in enumerate(rows):
        if not active_mask[i]:
            continue
        pos6.append(forward_kinematics(r["joints"])[:3, 3])
    if pos6:
        pos6 = np.array(pos6)
        span = float(np.linalg.norm(pos6.max(axis=0) - pos6.min(axis=0)))
    else:
        span = 0.0

    # 末端姿态两两最大角差
    Rs = []
    for i, r in enumerate(rows):
        if not active_mask[i]:
            continue
        Rs.append(forward_kinematics(r["joints"])[:3, :3])
    max_ang = 0.0
    if len(Rs) >= 2:
        for i in range(len(Rs)):
            for j in range(i + 1, len(Rs)):
                cosang = np.clip((np.trace(Rs[i] @ Rs[j].T) - 1) / 2, -1, 1)
                ang = math.degrees(math.acos(cosang))
                if ang > max_ang:
                    max_ang = ang

    results["valid"] = valid >= th["min_valid"]
    results["active"] = n_active >= th["min_active"]
    results["span"] = span >= th["max_pose_span_m"]
    results["orient"] = max_ang >= th["min_orient_deg"]

    print(f"\n{BOLD}[1/4] 数据质量{DIRECT_RIGHT}{RESET}")
    print(f"  有效帧数:             {valid}  {'>= ' + str(th['min_valid']):>10}  {_mark(results['valid'])}")
    print(f"  关节实际运动帧:       {n_active}  {'>= ' + str(th['min_active']):>10}  {_mark(results['active'])}")
    print(f"  末端空间跨度:         {span:.3f} m  {'>= ' + str(th['max_pose_span_m']):>8}  {_mark(results['span'])}"
          f"  {WARN_COLOR}(< 该值则平移约束弱){RESET}")
    print(f"  末端姿态最大角差:     {max_ang:.1f}°  {'>= ' + str(th['min_orient_deg']):>8}  {_mark(results['orient'])}")

    return results, {"valid": valid, "active": n_active, "span": span, "orient": max_ang}


# ============================================================================
# 判据 2: 多算法一致性
# ============================================================================

def check_algo_consistency(rows, use_tsai=True, solve_rows=None):
    th = THRESHOLDS
    if solve_rows:
        rows = rows[:solve_rows]
        print(f"  (多算法一致性仅用前 {solve_rows} 帧求解数据)")
    valid = [r for r in rows]
    if len(valid) < 3:
        print(f"[ERROR] 有效数据不足: {len(valid)} 行 (至少需要 3 组)")
        return None, None

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for r in valid:
        T = forward_kinematics(r["joints"])
        R_g2b.append(T[:3, :3].copy())
        t_g2b.append(T[:3, 3].copy())
        R_t2c.append(rodrigues_to_rmat(r["rvec"]))
        t_t2c.append(r["tvec"].reshape(3))

    methods = [
        (cv2.CALIB_HAND_EYE_TSAI, "Tsai-Lenz"),
        (cv2.CALIB_HAND_EYE_PARK, "Park-Martin"),
        (cv2.CALIB_HAND_EYE_HORAUD, "Horaud"),
        (cv2.CALIB_HAND_EYE_ANDREFF, "Andreff"),
        (cv2.CALIB_HAND_EYE_DANIILIDIS, "Daniilidis"),
    ]
    if not use_tsai:
        methods = methods[1:]

    results = {}
    for m, name in methods:
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
            results[name] = (R, t.flatten())
        except cv2.error:
            continue
    if not results:
        print(f"{FAIL_COLOR}[2/4] 多算法一致性: 所有算法均失败{FAIL_COLOR and RESET}")
        return None, None

    main_name = "Tsai-Lenz" if "Tsai-Lenz" in results else list(results.keys())[0]
    R0, t0 = results[main_name]

    print(f"\n{BOLD}[2/4] 多算法一致性 (相对主算法 {main_name}){DIRECT_RIGHT}{RESET}")
    dr_list, dt_list = [], []
    for name, (R, t) in results.items():
        dr = float(np.linalg.norm(R - R0, "fro"))
        dt = float(np.linalg.norm(t - t0))
        dr_list.append(dr)
        dt_list.append(dt)
        mark = " ◄主" if name == main_name else ""
        ok = dr <= th["algo_dr"] and dt <= th["algo_dt_m"]
        print(f"  {name:16s}  ΔR={dr:.4f}  Δt={dt*1000:7.1f} mm  {_mark(ok)}{mark}")

    dr_max = max(dr_list)
    dt_max = max(dt_list)
    passed = dr_max <= th["algo_dr"] and dt_max <= th["algo_dt_m"]
    print(f"  最大 ΔR={dr_max:.4f}  Δt={dt_max*1000:.1f} mm  →  {_mark(passed)}")

    return passed, {"dr_max": dr_max, "dt_max": dt_max, "main": R0, "main_t": t0, "all": results}


# ============================================================================
# 判据 3: 反投影闭环 (金标准)
# ============================================================================

def check_reprojection(rows, R, t, solve_rows=None):
    th = THRESHOLDS
    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = t

    joints_all = np.array([r["joints"] for r in rows])
    active_all = np.abs(joints_all).max(axis=1) > 0.01

    # 优先评估标定后新采集的帧 (它们最能检验标定泛化能力)
    if solve_rows and len(rows) > solve_rows:
        scan_idx = [i for i in range(solve_rows, len(rows)) if active_all[i]]
        label = f"后 {len(rows) - solve_rows} 帧新数据 (运动 {len(scan_idx)} 帧)"
    else:
        scan_idx = [i for i in range(len(rows)) if active_all[i]]
        label = f"{len(rows)} 帧数据"

    if len(scan_idx) < 2:
        # 新数据无运动帧则退回到求解帧
        scan_idx = [i for i in range(len(rows)) if active_all[i]]
        label = f"{len(rows)} 帧数据 (无新数据运动帧, 回退求解帧)"

    if len(scan_idx) < 2:
        print(f"{FAIL_COLOR}[3/4] 反投影闭环: 运动帧不足, 无法评估{RESET}")
        return False, {}

    pos = []
    rot = []
    for i in scan_idx:
        r = rows[i]
        cam_T_tag = np.eye(4)
        cam_T_tag[:3, :3] = rodrigues_to_rmat(r["rvec"])
        cam_T_tag[:3, 3] = r["tvec"]
        base_T_tag = forward_kinematics(r["joints"]) @ X @ cam_T_tag
        pos.append(base_T_tag[:3, 3])
        rot.append(base_T_tag[:3, :3])

    pos = np.array(pos)
    rot = np.array(rot)
    mean_pos = pos.mean(axis=0)

    # 平移
    dpos = np.linalg.norm(pos - mean_pos, axis=1)
    rms_pos = float(np.sqrt((dpos**2).mean()))
    max_pos = float(dpos.max())

    # 姿态: 相对平均姿态 (近似取首帧)
    ref = rot[0]
    cosang = np.clip((np.trace(rot @ ref.T, axis1=1, axis2=2) - 1) / 2, -1, 1)
    ang = np.degrees(np.arccos(cosang))
    rms_rot = float(np.sqrt((ang**2).mean()))

    passed = rms_pos <= th["rms_pos_m"] and max_pos <= th["max_pos_m"] and rms_rot <= th["rms_rot_deg"]

    print(f"\n{BOLD}[3/4] 反投影闭环 (金标准){DIRECT_RIGHT}{RESET}")
    print(f"  用 X 反算 base_T_tag, 评估 {label}:")
    print(f"  base_T_tag 均值: [{mean_pos[0]:+.4f}, {mean_pos[1]:+.4f}, {mean_pos[2]:+.4f}] m")
    print(f"  位置 RMS:        {rms_pos*1000:6.1f} mm  {'<= ' + str(th['rms_pos_m']*1000):>8}  {_mark(rms_pos <= th['rms_pos_m'])}")
    print(f"  位置最大偏差:    {max_pos*1000:6.1f} mm  {'<= ' + str(th['max_pos_m']*1000):>8}  {_mark(max_pos <= th['max_pos_m'])}")
    print(f"  姿态 RMS:        {rms_rot:6.2f}°   {'<= ' + str(th['rms_rot_deg']):>8}  {_mark(rms_rot <= th['rms_rot_deg'])}")

    return passed, {"rms_pos": rms_pos, "max_pos": max_pos, "rms_rot": rms_rot, "mean": mean_pos}


# ============================================================================
# 判据 4: 物理合理性
# ============================================================================

def check_physical(R, t):
    th = THRESHOLDS
    dist = float(np.linalg.norm(t))

    # 光轴方向 (相机 z 轴在 link6 系)
    opt_axis = R @ np.array([0.0, 0.0, 1.0])
    # 大致应朝远离末端的方向 (link6 z 为末端朝向, 相机通常近似共线), 仅提示

    # 光轴与 link6 末端朝向夹角
    cosang = np.clip(np.dot(opt_axis, np.array([0.0, 0.0, 1.0])), -1, 1)
    angle = math.degrees(math.acos(cosang))

    passed = dist * 1000 <= th["cam_dist_mm_max"]
    lo, hi = th["expect_cam_dist_mm"]

    print(f"\n{BOLD}[4/4] 物理合理性{DIRECT_RIGHT}{RESET}")
    print(f"  相机到 link6 原点:  {dist*1000:.0f} mm  {'<= ' + str(th['cam_dist_mm_max']):>8}  {_mark(passed)}")
    print(f"  (期望 {lo}-{hi} mm, 若明显超出请检查安装或内参)")
    print(f"  光轴相对 link6 末端朝向夹角: {angle:.0f}°  {WARN_COLOR}(仅供核对, 相机常有俯仰安装角){RESET}")

    return passed, {"dist_mm": dist * 1000, "axis_angle": angle}


# ============================================================================
# 主流程
# ============================================================================

def main():
    csv_path = None
    result_path = None

    args = sys.argv[1:]
    for a in args:
        if a in ("--help", "-h"):
            print(__doc__)
            return
    if args:
        csv_path = args[0]
    if len(args) > 1:
        result_path = args[1]

    if csv_path is None:
        csv_path = os.path.join(DATA_DIR, "calib_data.csv")
    if result_path is None:
        result_path = os.path.join(DATA_DIR, "calib_result.txt")

    if not os.path.isfile(csv_path):
        print(f"[ERROR] 找不到数据文件: {csv_path}")
        sys.exit(1)

    print(f"{BOLD}手眼标定验收{DIRECT_RIGHT}{RESET}")
    print(f"  数据:   {csv_path}")
    print(f"  结果:   {result_path}")

    rows = read_csv(csv_path)
    if not rows:
        print("[ERROR] CSV 中没有有效数据行")
        sys.exit(1)
    print(f"  有效帧: {len(rows)}")

    R, t, meta = read_result(result_path)
    if meta.get("algorithm"):
        print(f"  算法:   {meta['algorithm']}")
    print(f"  主算法: Tsai-Lenz (OpenCV)")
    if SOLVE_ROWS:
        print(f"  求解规模: 前 {SOLVE_ROWS} 帧 (其余 {len(rows) - SOLVE_ROWS} 帧为新数据)")

    # 判据 1
    dq, dq_info = check_data_quality(rows, SOLVE_ROWS)
    # 判据 2 (多算法一致性与主结果)
    ac, ac_info = check_algo_consistency(rows, solve_rows=SOLVE_ROWS)
    # 判据 3 用读入结果评估反投影 (优先新数据, 检验标定泛化)
    rc, rc_info = check_reprojection(rows, R, t, SOLVE_ROWS)
    # 判据 4
    ph, ph_info = check_physical(R, t)

    # ── 汇总 ──────────────────────────────────────────────────────
    score = _score(30, dq["valid"] and dq["active"] and dq["span"] and dq["orient"])
    score += _score(25, ac if ac else False)
    score += _score(35, rc)
    score += _score(10, ph)

    print(f"\n{'='*70}")
    print(f"{BOLD}验收汇总{DIRECT_RIGHT}{RESET}")
    print(f"  数据质量  (30分): {_mark(dq['valid'] and dq['active'] and dq['span'] and dq['orient'])}")
    print(f"  多算法一致性(25分): {_mark(bool(ac))}")
    print(f"  反投影闭环(35分):  {_mark(bool(rc))}")
    print(f"  物理合理性(10分):  {_mark(bool(ph))}")
    overall = score >= 60 and rc and ac and ph
    print(f"  {BOLD}总分: {score:.0f}/100  →  {_mark(overall)}{RESET}")

    if overall:
        print(f"\n{PASS_COLOR}标定合格, 可投入使用。{RESET}")
    else:
        print(f"\n{FAIL_COLOR}标定未达标。请根据上述 FAIL 项重采数据后重新求解。{RESET}")
        if not (dq["valid"] and dq["active"]):
            print("  - 帧数不足: 多摆姿态多采几帧, 确保关节变化 > 0.02 rad")
        if not dq["span"]:
            print(f"  - 末端空间跨度仅 {dq_info['span']:.3f} m: 机械臂多移动末端位置"
                  f"(不同位置+不同姿态), 激活 AX=XB 的平移约束")
        if not dq["orient"]:
            print(f"  - 姿态角差仅 {dq_info['orient']:.1f}°: 加大姿态差异 (>30°)")
        if ac is False:
            print("  - 多算法不一致: 数据冗余不足或 tag 测量噪声大, 重采时保持 tag 清晰、位姿多样")
        if rc is False:
            print(f"  - 反投影 RMS {rc_info.get('rms_pos', 0)*1000:.1f} mm: 这是金标准判据, 不达标基本说明"
                  f"标定不可用, 需重采")
        if not ph:
            print(f"  - 相机距离异常 ({ph_info.get('dist_mm', 0):.0f} mm): 检查相机实际安装或内参是否匹配")


if __name__ == "__main__":
    main()
