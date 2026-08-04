"""把 BNO08x 四元数转换为相对初始姿态的腕部载体姿态轨迹.

默认优先使用 6 轴 Game Rotation Vector (`quat_game_wxyz`); 旧数据没有该字段时
回退到 9 轴 Rotation Vector (`quat_wxyz`). 四元数是主结果, 欧拉角只用于显示.

示例:
  python3 orientation_trajectory.py data/output/bilateral/imu_left_bilateral.npz
  python3 orientation_trajectory.py imu_right.npz --side right --baseline-s 2
  python3 orientation_trajectory.py imu_left.npz --axis-map=-y,+x,+z

`--axis-map` 描述输出 X/Y/Z 轴分别来自哪个传感器轴. 默认 `+x,+y,+z`.
只允许右手坐标系(旋转矩阵行列式 +1), 防止用镜像矩阵破坏四元数旋转性质.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


def normalize_quaternions(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q, dtype=np.float64)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"四元数必须是 (N,4), 实际为 {q.shape}")
    norms = np.linalg.norm(q, axis=1)
    if np.any(~np.isfinite(q)) or np.any(norms < 1e-8):
        raise ValueError("四元数含非有限值或零范数")
    return q / norms[:, None], norms


def enforce_quaternion_continuity(q: np.ndarray) -> np.ndarray:
    """消除 q 与 -q 表示同一姿态造成的符号跳变."""
    out = np.array(q, dtype=np.float64, copy=True)
    for i in range(1, len(out)):
        if np.dot(out[i - 1], out[i]) < 0:
            out[i] *= -1.0
    return out


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    out = np.array(q, dtype=np.float64, copy=True)
    out[..., 1:] *= -1.0
    return out


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, 四元数顺序均为 [w,x,y,z]."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def parse_axis_map(spec: str) -> np.ndarray:
    """解析 `+x,+y,+z`: 输出坐标分量 = M @ 传感器坐标分量."""
    tokens = [token.strip().lower() for token in spec.split(",")]
    if len(tokens) != 3:
        raise ValueError("axis-map 必须有 3 项, 例如 +x,+y,+z")

    matrix = np.zeros((3, 3), dtype=np.float64)
    used = []
    axes = {"x": 0, "y": 1, "z": 2}
    for row, token in enumerate(tokens):
        if len(token) not in (1, 2) or token[-1] not in axes:
            raise ValueError(f"无效轴映射: {token}")
        sign = -1.0 if token.startswith("-") else 1.0
        axis = token[-1]
        used.append(axis)
        matrix[row, axes[axis]] = sign

    if sorted(used) != ["x", "y", "z"]:
        raise ValueError("axis-map 的 x/y/z 必须各使用一次")
    if not np.isclose(np.linalg.det(matrix), 1.0):
        raise ValueError("axis-map 必须保持右手坐标系(行列式 +1), 不能是镜像变换")
    return matrix


def quaternion_to_euler_zyx(q: np.ndarray) -> np.ndarray:
    """返回 ZYX 分解下 [roll_x, pitch_y, yaw_z], 单位弧度."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    roll = np.arctan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def quaternion_to_axis_angle(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """返回最短旋转轴和角度(弧度, 0..pi)."""
    shortest = np.array(q, dtype=np.float64, copy=True)
    shortest[shortest[:, 0] < 0] *= -1.0
    w = np.clip(shortest[:, 0], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    sin_half = np.sqrt(np.maximum(1.0 - w * w, 0.0))
    axis = np.zeros((len(q), 3), dtype=np.float64)
    valid = sin_half > 1e-8
    axis[valid] = shortest[valid, 1:] / sin_half[valid, None]
    axis[~valid, 0] = 1.0
    return axis, angle


def _select_quaternion_source(data: dict, source: str) -> tuple[str, np.ndarray, np.ndarray]:
    game = data.get("quat_game_wxyz")
    rotation = data.get("quat_wxyz")

    if source == "game":
        if game is None:
            raise ValueError("输入文件没有 quat_game_wxyz; 旧数据请使用 --source rotation")
        key, q, accuracy = "quat_game_wxyz", game, data.get("quat_game_accuracy")
    elif source == "rotation":
        if rotation is None:
            raise ValueError("输入文件没有 quat_wxyz")
        key, q, accuracy = "quat_wxyz", rotation, data.get("quat_accuracy")
    elif game is not None:
        game_array = np.asarray(game)
        if game_array.ndim == 2 and game_array.shape[1] == 4 and len(game_array):
            key, q, accuracy = "quat_game_wxyz", game, data.get("quat_game_accuracy")
        elif rotation is not None:
            key, q, accuracy = "quat_wxyz", rotation, data.get("quat_accuracy")
        else:
            raise ValueError("输入文件没有可用四元数")
    elif rotation is not None:
        key, q, accuracy = "quat_wxyz", rotation, data.get("quat_accuracy")
    else:
        raise ValueError("输入文件没有可用四元数")

    q = np.asarray(q, dtype=np.float64)
    if accuracy is None or len(accuracy) != len(q):
        accuracy = np.zeros(len(q), dtype=np.uint8)
    return key, q, np.asarray(accuracy, dtype=np.uint8)


def _select_timestamps(data: dict, n: int) -> tuple[str, np.ndarray]:
    for key in ("timestamps_sensor_clock", "timestamps"):
        value = data.get(key)
        if value is None:
            continue
        ts = np.asarray(value, dtype=np.float64)
        if len(ts) == n and np.all(np.isfinite(ts)) and np.all(np.diff(ts) >= 0):
            return key, ts
    rate = float(np.asarray(data.get("sample_rate_approx", 100)).reshape(-1)[0])
    return "sample_rate_fallback", np.arange(n, dtype=np.float64) / rate


def _quat_to_rotmat(q) -> np.ndarray:
    """四元数 [w,x,y,z] -> 3x3 (v_world = R @ v_sensor)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def compute_position_trajectory(q_arr: np.ndarray, data: dict,
                                timestamps: np.ndarray,
                                baseline_mask: np.ndarray,
                                static_accel: float = 0.25) -> np.ndarray:
    """6DOF 位置部分: 由线性加速度航位推算位置 (世界系, 有漂移).

    优先用硬件 linaccel_xyz (已去重力); 否则用 accel_xyz 减世界系重力.
    静止段估偏置并减掉, |a|<static_accel 判静止(ZUPT, 冻结), 否则积分.
    返回 (N,3) 位置 (米).
    """
    n = len(q_arr)
    pos = np.zeros((n, 3), dtype=np.float64)
    if n < 3:
        return pos

    lacc = data.get("linaccel_xyz")
    accel = data.get("accel_xyz")
    qm = np.asarray(q_arr, dtype=np.float64)

    if lacc is not None and np.asarray(lacc).shape == (n, 3):
        a_world = np.array([_quat_to_rotmat(qm[i]) @ np.asarray(lacc[i], dtype=np.float64)
                            for i in range(n)])
    elif accel is not None and np.asarray(accel).shape == (n, 3):
        a_world = np.array([_quat_to_rotmat(qm[i]) @ np.asarray(accel[i], dtype=np.float64)
                            for i in range(n)])
        g = a_world[baseline_mask].mean(axis=0) if baseline_mask.sum() > 1 else a_world.mean(axis=0)
        a_world = a_world - g
    else:
        return pos

    # 静止段偏置
    bias = a_world[baseline_mask].mean(axis=0) if baseline_mask.sum() > 1 else np.zeros(3)
    a_world = a_world - bias

    dt = np.diff(timestamps)
    dt = np.clip(dt, 1e-4, 0.05)
    vel = np.zeros(3, dtype=np.float64)
    for i in range(1, n):
        a = a_world[i]
        pos[i] = pos[i - 1]
        if np.linalg.norm(a) < static_accel:
            vel[:] = 0.0
        else:
            vel = (vel + a * dt[i - 1]) * float(np.exp(-dt[i - 1] / 5.0))
            pos[i] = pos[i - 1] + vel * dt[i - 1]
    return pos


def build_orientation_trajectory(data: dict, source: str = "auto",
                                 baseline_s: float = 1.0,
                                 axis_map: str = "+x,+y,+z") -> tuple[dict, dict]:
    source_key, q_raw, accuracy = _select_quaternion_source(data, source)
    q, input_norms = normalize_quaternions(q_raw)
    q = enforce_quaternion_continuity(q)
    n = len(q)
    if n < 2:
        raise ValueError("至少需要 2 个姿态样本")

    time_key, timestamps = _select_timestamps(data, n)
    elapsed = timestamps - timestamps[0]
    baseline_mask = elapsed <= max(float(baseline_s), 0.0)
    if baseline_mask.sum() < 2:
        baseline_mask[:min(10, n)] = True

    q_reference, _ = normalize_quaternions(np.mean(q[baseline_mask], axis=0, keepdims=True))
    q_relative = quaternion_multiply(
        np.repeat(quaternion_conjugate(q_reference), n, axis=0), q)
    q_relative, _ = normalize_quaternions(q_relative)
    q_relative = enforce_quaternion_continuity(q_relative)

    basis = parse_axis_map(axis_map)
    q_mapped = q_relative.copy()
    q_mapped[:, 1:] = q_relative[:, 1:] @ basis.T
    q_mapped, _ = normalize_quaternions(q_mapped)

    euler_rad = quaternion_to_euler_zyx(q_mapped)
    euler_deg = np.degrees(euler_rad)
    euler_unwrapped_deg = np.degrees(np.unwrap(euler_rad, axis=0))
    rotation_axis, rotation_angle = quaternion_to_axis_angle(q_mapped)

    gyro = np.asarray(data.get("gyro_xyz", np.zeros((n, 3))), dtype=np.float64)
    if gyro.shape != (n, 3):
        gyro = np.zeros((n, 3), dtype=np.float64)
    gyro_mapped_dps = np.degrees(gyro @ basis.T)

    delta_q = quaternion_multiply(quaternion_conjugate(q_mapped[:-1]), q_mapped[1:])
    _, step_angle = quaternion_to_axis_angle(delta_q)
    step_angle_deg = np.r_[0.0, np.degrees(step_angle)]
    dt = np.diff(timestamps)
    positive_dt = dt[dt > 0]
    burst_fraction = float(np.mean(dt < 0.001)) if len(dt) else 0.0
    step_p99_deg = float(np.percentile(step_angle_deg[1:], 99))
    step_max_deg = float(np.max(step_angle_deg))

    quality_warnings = []
    if time_key == "timestamps" and burst_fraction > 0.05:
        quality_warnings.append(
            "PC receive timestamps are bursty; use new recordings with timestamps_sensor_clock "
            "for angular timing and resampling.")
    if step_max_deg > 30.0:
        quality_warnings.append(
            "A consecutive orientation step exceeds 30 deg; inspect packet gaps, fast motion, "
            "or 9-axis magnetic correction before using derivatives.")

    # ---- 6DOF 位置部分: 由加速度航位推算 (世界系, 有漂移, 后续 SLAM 修正) ----
    pos_xyz = compute_position_trajectory(q, data, timestamps, baseline_mask)

    result = {
        "timestamps": timestamps,
        "elapsed_s": elapsed,
        "quat_source_wxyz": q,
        "quat_relative_wxyz": q_mapped,
        "pos_xyz": pos_xyz,          # 6DOF 位置 (米, 航位推算)
        "roll_x_deg": euler_deg[:, 0],
        "pitch_y_deg": euler_deg[:, 1],
        "yaw_z_deg": euler_deg[:, 2],
        "roll_x_unwrapped_deg": euler_unwrapped_deg[:, 0],
        "pitch_y_unwrapped_deg": euler_unwrapped_deg[:, 1],
        "yaw_z_unwrapped_deg": euler_unwrapped_deg[:, 2],
        "rotation_axis_xyz": rotation_axis,
        "rotation_angle_deg": np.degrees(rotation_angle),
        "step_angle_deg": step_angle_deg,
        "gyro_xyz_dps": gyro_mapped_dps,
        "accuracy": accuracy,
        "reference_quat_wxyz": q_reference[0],
    }
    metadata = {
        "source_quaternion": source_key,
        "time_source": time_key,
        "baseline_s": float(baseline_s),
        "baseline_samples": int(baseline_mask.sum()),
        "axis_map": axis_map,
        "euler_convention": "ZYX decomposition; output roll-X, pitch-Y, yaw-Z",
        "quaternion_convention": "Hamilton [w,x,y,z]; relative = conjugate(reference) * current",
        "quat_norm_max_error": float(np.max(np.abs(input_norms - 1.0))),
        "step_angle_p99_deg": step_p99_deg,
        "step_angle_max_deg": step_max_deg,
        "median_dt_s": float(np.median(positive_dt)) if len(positive_dt) else None,
        "timestamp_burst_fraction_dt_lt_1ms": burst_fraction,
        "quality_warnings": quality_warnings,
        "warning": (
            "Euler angles are display fields and may be singular near pitch +/-90 deg. "
            "Use quat_relative_wxyz for training and rotation composition."
        ),
    }
    return result, metadata


def load_npz(path: Path) -> dict:
    with np.load(path, allow_pickle=True) as npz:
        return {key: npz[key] for key in npz.files}


def save_trajectory(output_npz: Path, result: dict, metadata: dict,
                    source_path: Path, side: str) -> tuple[Path, Path, Path]:
    output_npz.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output_npz.with_name(output_npz.stem + ".csv")
    meta_path = output_npz.with_name(output_npz.stem + "_meta.json")

    np.savez_compressed(output_npz, **result)
    q = result["quat_relative_wxyz"]
    axis = result["rotation_axis_xyz"]
    gyro = result["gyro_xyz_dps"]
    pos = result["pos_xyz"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "qw", "qx", "qy", "qz",
            "roll_x_deg", "pitch_y_deg", "yaw_z_deg",
            "roll_x_unwrapped_deg", "pitch_y_unwrapped_deg", "yaw_z_unwrapped_deg",
            "rotation_axis_x", "rotation_axis_y", "rotation_axis_z", "rotation_angle_deg",
            "step_angle_deg", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "accuracy",
            "pos_x", "pos_y", "pos_z",
        ])
        for i in range(len(q)):
            writer.writerow([
                f"{result['timestamps'][i]:.9f}", f"{result['elapsed_s'][i]:.6f}",
                *[f"{v:.8f}" for v in q[i]],
                f"{result['roll_x_deg'][i]:.6f}",
                f"{result['pitch_y_deg'][i]:.6f}",
                f"{result['yaw_z_deg'][i]:.6f}",
                f"{result['roll_x_unwrapped_deg'][i]:.6f}",
                f"{result['pitch_y_unwrapped_deg'][i]:.6f}",
                f"{result['yaw_z_unwrapped_deg'][i]:.6f}",
                *[f"{v:.8f}" for v in axis[i]],
                f"{result['rotation_angle_deg'][i]:.6f}",
                f"{result['step_angle_deg'][i]:.6f}",
                *[f"{v:.6f}" for v in gyro[i]], int(result["accuracy"][i]),
                f"{pos[i][0]:.6f}", f"{pos[i][1]:.6f}", f"{pos[i][2]:.6f}",
            ])

    meta = {
        "source_file": str(source_path),
        "side": side,
        "n_samples": int(len(q)),
        **metadata,
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return output_npz, csv_path, meta_path


def main() -> int:
    parser = argparse.ArgumentParser(description="BNO08x 四元数转腕部载体姿态轨迹")
    parser.add_argument("input", type=Path, help="imu_*.npz 输入文件")
    parser.add_argument("-o", "--output", type=Path, help="输出 NPZ 路径")
    parser.add_argument("--source", choices=("auto", "game", "rotation"), default="auto",
                        help="auto 优先 6轴 game, 旧数据回退 9轴 rotation")
    parser.add_argument("--baseline-s", type=float, default=1.0,
                        help="开头静止归零时长, 默认 1 秒")
    parser.add_argument("--axis-map", default="+x,+y,+z",
                        help="输出 X/Y/Z 对应的传感器轴, 例如 -y,+x,+z")
    parser.add_argument("--side", choices=("left", "right", "unknown"), default="unknown")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"[ERR] 输入文件不存在: {args.input}", file=sys.stderr)
        return 1
    output = args.output or args.input.with_name(args.input.stem + "_orientation.npz")
    try:
        result, metadata = build_orientation_trajectory(
            load_npz(args.input), source=args.source,
            baseline_s=args.baseline_s, axis_map=args.axis_map)
        paths = save_trajectory(output, result, metadata, args.input, args.side)
    except (ValueError, OSError) as exc:
        print(f"[ERR] {exc}", file=sys.stderr)
        return 1

    print(f"[OK] 姿态轨迹: {len(result['timestamps'])} 样本")
    print(f"     四元数源: {metadata['source_quaternion']}")
    print(f"     时间源:   {metadata['time_source']}")
    for warning in metadata["quality_warnings"]:
        print(f"[WARN] {warning}")
    for path in paths:
        print(f"     {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
