#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 腕部解剖轴标定辅助脚本。

读取 orientation_trajectory.py 生成的 *_orientation.csv（必须在 sensor-frame
即 axis-map=+x,+y,+z 下生成），自动推断左右手应使用的 axis-map 字符串。

用法:
    python calibrate_imu_axis.py \
        data/output/axis_calib_left_03/imu_left_axis_calib_left_03_orientation.csv \
        --side left

    # 自动写回 imu_client.py
    python calibrate_imu_axis.py \
        data/output/axis_calib_left_03/imu_left_axis_calib_left_03_orientation.csv \
        --side left --apply

输出示例:
    [INFO] 标定数据: 17670 样本, 89.4 s
    [INFO] 检测到 6 组动作
    [OK] 推荐左手 axis-map: +x,-y,-z
    [OK] 右手系校验通过 (det=+1)
    [OK] 已更新 imu_client.py: ORIENTATION_AXIS_MAPS['left'] = "+x,-y,-z"
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# 把本文件所在目录加入路径，以便复用 orientation_trajectory.parse_axis_map
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from orientation_trajectory import parse_axis_map  # noqa: E402

MOTION_NAMES = ["pron", "sup", "up", "down", "left", "right"]
AXIS_LABELS = ["roll_x", "pitch_y", "yaw_z"]
AXIS_LETTER = ["x", "y", "z"]
DEFAULT_IMU_CLIENT = SCRIPT_DIR / "imu_client.py"
SIGN_CONFIDENCE_THRESHOLD = 0.2  # |median|/peak, 低于此值认为符号不可靠


def load_orientation_csv(path: Path) -> dict:
    """读取 orientation CSV,返回计算所需数组."""
    required = [
        "elapsed_s",
        "roll_x_unwrapped_deg",
        "pitch_y_unwrapped_deg",
        "yaw_z_unwrapped_deg",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
    ]
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError("CSV 没有表头")
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV 缺少列: {missing}")
        for row in reader:
            rows.append({k: float(row[k]) for k in required})

    return {k: np.array([r[k] for r in rows]) for k in required}


def moving_average(x: np.ndarray, window_s: float, sr: float) -> np.ndarray:
    """简单滑动平均,窗口时长 window_s 秒."""
    k = max(1, int(round(window_s * sr)))
    if k > len(x):
        return x.copy()
    pad = k // 2
    xp = np.pad(x, (pad, pad), mode="edge")
    return np.convolve(xp, np.ones(k) / k, mode="valid")[: len(x)]


def detect_motion_groups(
    t: np.ndarray,
    gyro_mag: np.ndarray,
    sr: float,
) -> list[tuple[int, int]]:
    """
    检测 6 个动作大组(旋前/旋后/上/下/左/右)。
    使用陀螺仪模长,合并组内短停顿(约 1 s),但保留组间 1.5 s 以上间隔。
    """
    mag_smooth = moving_average(gyro_mag, window_s=0.3, sr=sr)
    # 阈值: 取 p30 附近,限制在合理范围
    threshold = float(np.percentile(mag_smooth, 30))
    threshold = min(max(threshold, 5.0), 10.0)

    active = mag_smooth > threshold
    min_dur = int(round(1.0 * sr))       # 每组至少 1 s
    max_gap = int(round(1.2 * sr))       # 组内停顿 < 1.2 s 时合并

    groups: list[tuple[int, int]] = []
    in_seg = False
    start = 0
    n = len(active)
    for i, a in enumerate(active):
        if a and not in_seg:
            start = i
            in_seg = True
        elif not a and in_seg:
            if i - start >= min_dur:
                if groups and start - groups[-1][1] <= max_gap:
                    groups[-1] = (groups[-1][0], i)
                else:
                    groups.append((start, i))
            in_seg = False
    if in_seg and n - start >= min_dur:
        if groups and start - groups[-1][1] <= max_gap:
            groups[-1] = (groups[-1][0], n - 1)
        else:
            groups.append((start, n - 1))

    return groups


def fixed_window_groups(
    t: np.ndarray,
    sr: float,
    n_groups: int = 6,
    baseline_s: float = 2.0,
    group_s: float = 15.0,
) -> list[tuple[int, int]]:
    """固定窗口回退: 假设 baseline_s 基线后每个大组 group_s 秒."""
    n = len(t)
    start_sample = min(int(round(baseline_s * sr)), n)
    step = int(round(group_s * sr))
    groups: list[tuple[int, int]] = []
    for i in range(n_groups):
        s = min(start_sample + i * step, n - 1)
        e = min(start_sample + (i + 1) * step, n - 1)
        if s >= n:
            break
        groups.append((s, e))
    return groups


def analyze_group(data: dict, s: int, e: int) -> dict:
    """分析一个大组: 找出主导轴、符号和幅值."""
    rx = data["roll_x_unwrapped_deg"][s:e]
    ry = data["pitch_y_unwrapped_deg"][s:e]
    rz = data["yaw_z_unwrapped_deg"][s:e]

    axes = [rx, ry, rz]
    medians = np.array([float(np.median(a)) for a in axes])
    peaks = np.array([float(np.max(np.abs(a))) for a in axes])
    dominant = int(np.argmax(peaks))
    median = medians[dominant]
    peak = peaks[dominant]
    sign_confidence = abs(median) / peak if peak > 1e-6 else 0.0

    return {
        "axis": dominant,
        "sign": "+" if median >= 0 else "-",
        "median": median,
        "peak": peak,
        "sign_confidence": sign_confidence,
        "all_medians": medians,
        "all_peaks": peaks,
    }


def infer_axis_map(
    data: dict,
    groups: list[tuple[int, int]],
    side: str,
) -> tuple[str, list[dict]]:
    """
    根据 6 个动作大组推断 axis-map。
    组顺序固定: pron/sup/up/down/left/right。

    约定(与 imu_client.py 当前配置一致):
      - 左手: +X=旋前, +Y=向上/屈, +Z=向右
      - 右手: +X=旋后, +Y=向上/屈, +Z=向右
    """
    if len(groups) < 6:
        raise ValueError(f"动作组数量不足: 需要 6, 实际 {len(groups)}")

    analyses = [analyze_group(data, s, e) for s, e in groups[:6]]

    # 根据手侧选择用于确定 X 轴正方向的动作
    x_ref_motion = 0 if side == "left" else 1  # left: pron, right: sup

    def build_token(ref_motion_idx: int) -> str:
        a = analyses[ref_motion_idx]
        return f"{a['sign']}{AXIS_LETTER[a['axis']]}"

    # X: 旋前(left) 或 旋后(right)
    token_x = build_token(x_ref_motion)
    # Y: 向上
    token_y = build_token(2)
    # Z: 向右 (向右是“向左”的反向)
    left_analysis = analyses[4]
    z_axis = left_analysis["axis"]
    z_sign = "+" if left_analysis["median"] < 0 else "-"
    token_z = f"{z_sign}{AXIS_LETTER[z_axis]}"

    axis_map = ",".join([token_x, token_y, token_z])
    return axis_map, analyses


def print_group_table(
    data: dict,
    groups: list[tuple[int, int]],
    analyses: list[dict],
) -> None:
    """打印每组信息,方便用户核对."""
    print("\n动作组分析(组内各轴峰值与中位数):")
    print(f"{'组':>3} {'动作':>6} {'t_start':>8} {'t_end':>8} {'dur':>6} {'主导轴':>10} {'峰值':>8} {'中位':>8} {'可靠度':>8}")
    for idx, ((s, e), label) in enumerate(zip(groups, MOTION_NAMES)):
        t0, t1 = data["elapsed_s"][s], data["elapsed_s"][e]
        a = analyses[idx]
        dom_label = AXIS_LABELS[a["axis"]]
        conf = a["sign_confidence"]
        conf_str = f"{conf:.2f}"
        print(f"{idx:3d} {label:>6} {t0:8.2f} {t1:8.2f} {t1-t0:6.2f} "
              f"{dom_label:>10} {a['peak']:8.1f} {a['median']:8.1f} {conf_str:>8}")


def optional_plot(
    data: dict,
    groups: list[tuple[int, int]],
    output_path: Path,
) -> None:
    """可选保存标定示意图."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[WARN] matplotlib 不可用,跳过绘图: {exc}")
        return

    t = data["elapsed_s"]
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(t, data["roll_x_unwrapped_deg"], label="roll_x")
    axes[0].plot(t, data["pitch_y_unwrapped_deg"], label="pitch_y")
    axes[0].plot(t, data["yaw_z_unwrapped_deg"], label="yaw_z")
    axes[0].set_ylabel("deg")
    axes[0].legend()
    axes[0].set_title("Calibration orientation (sensor frame)")

    axes[1].plot(t, data["gyro_x_dps"], label="gyro_x")
    axes[1].plot(t, data["gyro_y_dps"], label="gyro_y")
    axes[1].plot(t, data["gyro_z_dps"], label="gyro_z")
    axes[1].set_ylabel("dps")
    axes[1].legend()

    mag = np.sqrt(
        data["gyro_x_dps"] ** 2 + data["gyro_y_dps"] ** 2 + data["gyro_z_dps"] ** 2
    )
    axes[2].plot(t, mag, label="gyro magnitude")
    axes[2].set_ylabel("dps")
    axes[2].legend()

    axes[3].plot(t, data["roll_x_unwrapped_deg"], alpha=0.5)
    axes[3].plot(t, data["pitch_y_unwrapped_deg"], alpha=0.5)
    axes[3].plot(t, data["yaw_z_unwrapped_deg"], alpha=0.5)
    for s, e in groups:
        axes[3].axvspan(t[s], t[e], color="gray", alpha=0.2)
    axes[3].set_ylabel("deg")
    axes[3].set_xlabel("elapsed_s")
    axes[3].set_title("detected groups")

    for ax in axes:
        ax.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"[INFO] 标定示意图已保存: {output_path}")


def update_imu_client_axis_map(
    side: str,
    axis_map: str,
    imu_client_path: Path = DEFAULT_IMU_CLIENT,
) -> bool:
    """把推断出的 axis-map 写回 imu_client.py 的 ORIENTATION_AXIS_MAPS."""
    if not imu_client_path.exists():
        print(f"[ERR] 找不到 imu_client.py: {imu_client_path}", file=sys.stderr)
        return False

    text = imu_client_path.read_text(encoding="utf-8")
    # 匹配 "left": "..." 或 'left': '...'
    pattern = re.compile(
        rf'(["\']{re.escape(side)}["\']\s*:\s*)["\'][^"\']+["\']'
    )
    if not pattern.search(text):
        print(f"[ERR] 在 {imu_client_path} 中未找到 ORIENTATION_AXIS_MAPS['{side}']",
              file=sys.stderr)
        return False

    new_text, n = pattern.subn(rf'\1"{axis_map}"', text)
    if n != 1:
        print(f"[ERR] 替换失败,匹配到 {n} 处", file=sys.stderr)
        return False

    imu_client_path.write_text(new_text, encoding="utf-8")
    print(f"[OK] 已更新 {imu_client_path.name}: ORIENTATION_AXIS_MAPS['{side}'] = \"{axis_map}\"")
    return True


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="从 IMU 标定 CSV 自动推断 axis-map",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
动作顺序要求(固定): 旋前×3, 旋后×3, 向上×3, 向下×3, 向左×3, 向右×3。
CSV 必须是在 sensor-frame (axis-map=+x,+y,+z) 下生成的 orientation CSV。
""",
    )
    parser.add_argument("csv", type=Path, help="orientation CSV 文件路径")
    parser.add_argument("--side", choices=["left", "right"], default="left",
                        help="手侧,仅用于输出提示")
    parser.add_argument("--output-json", type=Path, default=None,
                        help="可选: 将结果保存为 JSON")
    parser.add_argument("--plot", type=Path, default=None,
                        help="可选: 保存标定示意图(需 matplotlib)")
    parser.add_argument("--fixed-window", action="store_true",
                        help="强制使用固定窗口(2s 基线 + 6×15s),不自动检测")
    parser.add_argument("--apply", action="store_true",
                        help="自动把推断结果写回 imu_client.py 的 ORIENTATION_AXIS_MAPS")
    parser.add_argument("--imu-client", type=Path, default=DEFAULT_IMU_CLIENT,
                        help=f"--apply 时写回的文件路径(默认: {DEFAULT_IMU_CLIENT})")
    args = parser.parse_args(argv)

    if not args.csv.exists():
        print(f"[ERR] 文件不存在: {args.csv}", file=sys.stderr)
        return 1

    try:
        data = load_orientation_csv(args.csv)
    except Exception as exc:
        print(f"[ERR] 读取 CSV 失败: {exc}", file=sys.stderr)
        return 1

    t = data["elapsed_s"]
    n = len(t)
    sr = float(n / (t[-1] - t[0])) if n > 1 and t[-1] > t[0] else 200.0
    print(f"[INFO] 标定数据: {n} 样本, {t[-1]-t[0]:.2f} s, 约 {sr:.1f} Hz")

    gyro_mag = np.sqrt(
        data["gyro_x_dps"] ** 2
        + data["gyro_y_dps"] ** 2
        + data["gyro_z_dps"] ** 2
    )

    if args.fixed_window:
        groups = fixed_window_groups(t, sr)
        print(f"[INFO] 使用固定窗口: {len(groups)} 组")
    else:
        groups = detect_motion_groups(t, gyro_mag, sr)
        print(f"[INFO] 检测到 {len(groups)} 组动作")
        if len(groups) != 6:
            print(f"[WARN] 检测组数不是 6,改用固定窗口回退...")
            groups = fixed_window_groups(t, sr)
            print(f"[INFO] 固定窗口: {len(groups)} 组")

    if len(groups) < 6:
        print(f"[ERR] 最终组数 {len(groups)} 不足 6,无法推断 axis-map", file=sys.stderr)
        print("       请检查: 1) 动作幅度是否足够; 2) 是否按固定顺序做了 6 组动作; "
              "3) CSV 是否为 sensor-frame (+x,+y,+z); 4) 使用 --fixed-window 重试", file=sys.stderr)
        return 1

    try:
        axis_map, analyses = infer_axis_map(data, groups, args.side)
    except Exception as exc:
        print(f"[ERR] 推断 axis-map 失败: {exc}", file=sys.stderr)
        return 1

    print_group_table(data, groups, analyses)

    # 提示符号可靠度较低的组
    x_ref_motion = 0 if args.side == "left" else 1
    used_indices = {x_ref_motion, 2, 4}  # X参考、向上、向左
    low_conf = [
        i for i in used_indices
        if analyses[i]["sign_confidence"] < SIGN_CONFIDENCE_THRESHOLD
    ]
    if low_conf:
        print("\n[WARN] 以下用于推断 axis-map 的组符号可靠度较低(<0.2),建议核对或重采:")
        for i in sorted(low_conf):
            a = analyses[i]
            print(f"       - {MOTION_NAMES[i]}: 可靠度={a['sign_confidence']:.2f}, "
                  f"峰值={a['peak']:.1f}, 中位={a['median']:.1f}")

    # 校验右手系
    try:
        _ = parse_axis_map(axis_map)
        det_ok = True
    except Exception as exc:
        print(f"[ERR] axis-map 校验失败: {exc}", file=sys.stderr)
        det_ok = False

    print(f"\n[OK] 推荐{args.side}手 axis-map: {axis_map}")
    if det_ok:
        print("[OK] 右手系校验通过 (det=+1)")

    if args.apply:
        if not det_ok:
            print("[ERR] axis-map 未通过右手系校验,拒绝自动写入(--apply)",
                  file=sys.stderr)
            return 1
        if not update_imu_client_axis_map(args.side, axis_map, args.imu_client):
            print("[提示] 请手动更新 imu_client.py:")
            print(f'       ORIENTATION_AXIS_MAPS["{args.side}"] = "{axis_map}"')
            return 1
    else:
        print("[提示] 请更新 customer_collector/imu_client.py:")
        print(f'       ORIENTATION_AXIS_MAPS["{args.side}"] = "{axis_map}"')
        print("       或重跑时加 --apply 自动写入")

    if args.output_json:
        result = {
            "side": args.side,
            "axis_map": axis_map,
            "csv": str(args.csv),
            "n_groups": len(groups),
            "sample_rate_hz": round(sr, 1),
            "det_ok": det_ok,
        }
        args.output_json.write_text(json.dumps(result, ensure_ascii=False, indent=2),
                                    encoding="utf-8")
        print(f"[INFO] 结果已保存: {args.output_json}")

    if args.plot:
        optional_plot(data, groups, args.plot)

    return 0 if det_ok else 1


if __name__ == "__main__":
    sys.exit(main())
