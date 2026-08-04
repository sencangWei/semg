"""sEMG + IMU 多设备数据对齐工具.

后处理阶段使用, 把 sEMG 和 IMU 的数据按真实时间戳 (wall-clock) 对齐,
切分出任意时间窗口的数据段.

用法:
  python align.py session_2026_07_07_emg.npz imu_left.npz imu_right.npz
  python align.py --config session_dir/
  python align.py --segment 10.0 25.5          # 切 10.0s - 25.5s
  python align.py --events session_events.csv   # 按事件切分

输出:
  aligned_<session>_segment_<start>s_<end>s.npz
    sEMG (T_emg, 32), IMU_left (T_imu, features), IMU_right (T_imu, features)
    timestamps_emg, timestamps_imu_left, timestamps_imu_right
    sync_base_ts, segment_start, segment_end
    event_label

核心原理:
  1. 所有设备的时间戳都是真实 Unix 秒, 可以直接比较
  2. 按真实时间切分: 找 t_start <= ts < t_end 的所有样本
  3. 各设备采样率不同, 输出数组长度不同 (正常)
  4. 若需要帧级对齐 (如配对 sEMG 和 IMU 帧), 做 resample/interpolate
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import numpy as np


# =============================================================================
# 工具函数
# =============================================================================

def load_npz(path: Path) -> dict:
    """加载 npz, 返回 dict."""
    try:
        return dict(np.load(path, allow_pickle=True))
    except FileNotFoundError:
        print(f"[ERR] 文件不存在: {path}")
        sys.exit(1)


def print_npz_info(name: str, d: dict):
    """打印 npz 内容摘要."""
    print(f"\n[{name}]")
    for key, val in d.items():
        if (isinstance(val, np.ndarray) and val.ndim == 1 and len(val) > 5
                and np.issubdtype(val.dtype, np.number)):
            v0, v1, v_last = val[0], val[1], val[-1]
            print(f"  {key:30s} shape={val.shape} dtype={val.dtype}  "
                  f"range=[{v0:.4f} .. {v_last:.4f}]")
        else:
            print(f"  {key:30s} {repr(val)[:80]}")


def slice_by_time(ts: np.ndarray, t_start: float, t_end: float) -> np.ndarray:
    """返回 ts 中 t_start <= ts < t_end 的布尔掩码."""
    return (ts >= t_start) & (ts < t_end)


def slice_relative(elapsed: np.ndarray, t_start_rel: float, t_end_rel: float) -> np.ndarray:
    """返回相对时间 elapsed 中 t_start_rel <= elapsed < t_end_rel 的掩码."""
    return (elapsed >= t_start_rel) & (elapsed < t_end_rel)


def align_two_devices(
    ts_main: np.ndarray, data_main: np.ndarray,
    ts_other: np.ndarray, data_other: np.ndarray,
    t_start: float, t_end: float,
) -> tuple:
    """
    按真实时间窗口 [t_start, t_end] 对齐两组数据.

    Args:
        ts_main: 主设备时间戳 (Unix 秒), shape (T_main,)
        data_main: 主设备数据, shape (T_main, N_main)
        ts_other: 其他设备时间戳, shape (T_other,)
        data_other: 其他设备数据, shape (T_other, N_other)
        t_start, t_end: 真实时间窗口

    Returns:
        (mask_main, mask_other) 两个布尔掩码
    """
    mask_main = slice_by_time(ts_main, t_start, t_end)
    mask_other = slice_by_time(ts_other, t_start, t_end)
    return mask_main, mask_other


def cross_correlation_align(
    sig_emg: np.ndarray, sig_imu: np.ndarray,
    fs_emg: float = 1000.0, fs_imu: float = 100.0,
    max_lag_s: float = 5.0,
) -> float:
    """
    用加速度包络和 EMG RMS 做 cross-correlation 找精确时间偏移.

    Args:
        sig_emg: (T_emg,) EMG 通道
        sig_imu: (T_imu,) 加速度 magnitude (sqrt(ax^2+ay^2+az^2))
        fs_*: 采样率

    Returns:
        offset_s: sig_imu 比 sig_emg 早 offset_s 秒 (正值=IMU 早)

    方法:
        1. 下采样两者到公共频率
        2. 计算各自包络 (RMS / abs)
        3. 找 cross-correlation 峰值
    """
    # 下采样到 50Hz
    ds_factor_emg = max(1, int(fs_emg / 50))
    ds_factor_imu = max(1, int(fs_imu / 50))

    emg_ds = sig_emg[::ds_factor_emg]
    imu_ds = sig_imu[::ds_factor_imu]
    fs_ds = min(fs_emg / ds_factor_emg, fs_imu / ds_factor_imu)

    # 简单包络
    emg_env = np.abs(emg_ds)
    imu_env = np.abs(imu_ds)

    # 归一化
    emg_env = (emg_env - emg_env.mean()) / (emg_env.std() + 1e-8)
    imu_env = (imu_env - imu_env.mean()) / (imu_env.std() + 1e-8)

    # Cross-correlation
    corr = np.correlate(imu_env, emg_env, mode="full")
    lags = np.arange(-len(emg_env) + 1, len(imu_env))
    lag_s = lags / fs_ds

    # 限制范围
    mask = np.abs(lag_s) <= max_lag_s
    if mask.sum() == 0:
        return 0.0, 0.0, False

    corr_masked = np.where(mask, corr, -np.inf)
    best_lag_idx = int(np.argmax(corr_masked))
    offset = lag_s[best_lag_idx]
    corr_max = corr[best_lag_idx]
    n = min(len(emg_env), len(imu_env))
    corr_norm = float(corr_max / n) if n > 0 else 0.0   # 归一化相关系数 ~[-1,1]
    # 可靠性: 归一化相关 > 0.05 且 offset 没撞搜索边界(撞边界=没找到真峰)
    reliable = (corr_norm > 0.05) and (abs(offset) < max_lag_s * 0.95)
    print(f"  [对齐] offset={offset:.4f}s  corr_norm={corr_norm:.3f}  "
          f"{'可靠' if reliable else '不可靠(假对齐)'}")
    return offset, corr_max, reliable


# =============================================================================
# 对齐引擎
# =============================================================================

class MultiDeviceAligner:
    """
    多设备数据对齐器.

    用法:
        aligner = MultiDeviceAligner(sync_base_ts=1234567890.123)
        aligner.add_device("emg", emg_npz)
        aligner.add_device("imu_left", imu_left_npz)
        aligner.add_device("imu_right", imu_right_npz)
        result = aligner.slice(t_start=10.0, t_end=25.5)
    """

    def __init__(self, sync_base_ts: float = None):
        self.devices: dict[str, dict] = {}
        self.sync_base_ts = sync_base_ts
        self._global_offset: dict[str, float] = {}  # 各设备相对全局基准的偏移

    def add_device(self, name: str, npz_data: dict,
                   offset_s: float = 0.0):
        """
        添加一个设备的数据.

        Args:
            name: 设备名 (emg / imu_left / imu_right / camera)
            npz_data: 从 np.load() 得到的 dict
            offset_s: 该设备相对全局 sync_base_ts 的手动校准偏移 (秒)
                      例: IMU 比 sEMG 早 0.5s -> offset_s = -0.5
        """
        ts = npz_data.get("timestamps")
        if ts is None:
            ts = npz_data.get("elapsed_since_sync")
            if ts is not None and self.sync_base_ts:
                ts = ts + self.sync_base_ts

        self.devices[name] = {
            "npz": npz_data,
            "timestamps": ts,
            "offset_s": offset_s,
        }
        print(f"  [{name}] {len(ts)} 样本, "
              f"t=[{ts[0]:.3f} .. {ts[-1]:.3f}]")

    def auto_align(self, primary: str = "emg",
                   secondaries: list[str] = None,
                   cc_max_lag_s: float = 5.0):
        """
        自动对齐: 用 cross-correlation 找各设备相对 primary 的偏移.

        Args:
            primary: 主设备名 (通常是 emg)
            secondaries: 要对齐的其他设备列表
        """
        if primary not in self.devices:
            print(f"[WARN] 主设备 {primary} 不存在, 跳过 auto_align")
            return

        main = self.devices[primary]
        ts_main = main["timestamps"]
        data_main = main["npz"].get("data")

        if secondaries is None:
            secondaries = [n for n in self.devices if n != primary]

        for name in secondaries:
            if name not in self.devices:
                continue
            dev = self.devices[name]
            ts_other = dev["timestamps"]
            npz = dev["npz"]

            # 找两者的公共时间窗口
            t_overlap_start = max(ts_main[0], ts_other[0])
            t_overlap_end = min(ts_main[-1], ts_other[-1])
            if t_overlap_end <= t_overlap_start:
                print(f"  [{name}] 无公共时间窗口, 跳过")
                continue

            # 提取信号
            m_main = slice_by_time(ts_main, t_overlap_start, t_overlap_end)
            m_other = slice_by_time(ts_other, t_overlap_start, t_overlap_end)
            # 分臂对齐(按实测映射): 右手=通道1-8(ch0-7)、左手=通道9-16(ch8-15)
            # 右 IMU 对 sEMG ch0(右手); 左 IMU 对 sEMG ch8(左手)
            if data_main is not None:
                ch = 0 if "right" in name.lower() else 8
                sig_main = data_main[m_main][:, ch]
            else:
                sig_main = None

            # IMU 加速度 magnitude
            accel = npz.get("accel_xyz")
            if accel is not None:
                sig_other = np.sqrt(np.sum(accel[m_other] ** 2, axis=1))
                fs_main = float(npz.get("sample_rate", 1000))
                fs_other = float(npz.get("sample_rate_approx", 100))
                if sig_main is not None:
                    offset, corr_max, reliable = cross_correlation_align(
                        sig_main, sig_other,
                        fs_emg=fs_main, fs_imu=fs_other,
                        max_lag_s=cc_max_lag_s,
                    )
                    dev["aligned"] = reliable
                    if reliable:
                        dev["offset_s"] = -offset
                        print(f"  [{name}] 对齐成功 offset={-offset:.4f}s")
                    else:
                        dev["offset_s"] = 0.0
                        print(f"  [{name}] 对齐失败(corr低/撞边界=假offset), 不校正")
                else:
                    dev["aligned"] = False
                    print(f"  [{name}] 无 emg data 列可用, 跳过")
            else:
                print(f"  [{name}] 无 accel_xyz 字段, 跳过 auto_align")

    def slice(self, t_start: float, t_end: float) -> dict:
        """
        按真实时间窗口切分所有设备数据.

        Returns:
            dict: {device_name: {"timestamps": ..., "data": ..., "npz_keys": ...}}
        """
        result = {}
        for name, dev in self.devices.items():
            ts = dev["timestamps"]
            offset = dev["offset_s"]
            ts_corrected = ts - offset

            mask = slice_by_time(ts_corrected, t_start, t_end)
            n_sel = int(mask.sum())

            if n_sel == 0:
                print(f"[WARN] [{name}] 在 t=[{t_start}, {t_end}) 无数据")
                continue

            npz = dev["npz"]
            sliced = {}

            # 对齐 timestamps
            sliced["timestamps"] = ts_corrected[mask]
            sliced["elapsed_since_sync"] = ts_corrected[mask] - (self.sync_base_ts or ts_corrected[0])

            # 按字段名切分
            for key, val in npz.items():
                if val.ndim == 1 and len(val) == len(ts):
                    sliced[key] = val[mask]
                elif val.ndim == 2 and val.shape[0] == len(ts):
                    sliced[key] = val[mask]

            result[name] = {
                "n_samples": n_sel,
                "t_range": (float(ts_corrected[mask][0]), float(ts_corrected[mask][-1])),
                **sliced,
            }

            print(f"  [{name}] 切出 {n_sel} 样本, "
                  f"t=[{sliced['timestamps'][0]:.3f} .. {sliced['timestamps'][-1]:.3f}]")

        return result

    def slice_relative(self, t_start_rel: float, t_end_rel: float) -> dict:
        """按相对时间 (相对于 sync_base_ts) 切分."""
        if self.sync_base_ts is None:
            print("[ERR] sync_base_ts 未设置, 无法用相对时间切分")
            sys.exit(1)
        return self.slice(self.sync_base_ts + t_start_rel,
                         self.sync_base_ts + t_end_rel)

    def slice_by_events(self, events: list[dict],
                        event_label: str = None) -> dict:
        """
        按事件标记切分: 从第一个指定 event_label 到最后一个.

        Args:
            events: 从 events.csv 读出的事件列表
            event_label: 要切分的事件标签 (None=第一个事件到最后一个事件)
        """
        if not events:
            print("[ERR] 无事件")
            return {}

        if event_label:
            labeled = [e for e in events if e.get("label") == event_label]
            if len(labeled) < 2:
                print(f"[WARN] 事件标签 '{event_label}' 少于 2 个, 返回全段")
                labeled = events
        else:
            labeled = events

        t_start = min(float(e["wall_ts"]) for e in labeled)
        t_end = max(float(e["wall_ts"]) for e in labeled)
        return self.slice(t_start, t_end)


# =============================================================================
# 命令行接口
# =============================================================================

def cmd_info(npz_paths: list[Path]):
    """打印 npz 信息."""
    for p in npz_paths:
        d = load_npz(p)
        print_npz_info(p.stem, d)


def cmd_slice(npz_paths: list[Path], t_start: float, t_end: float,
              output: Path = None, sync_base_ts: float = None):
    """按时间窗口切分."""
    print(f"\n[切分] t=[{t_start}, {t_end})")

    aligner = MultiDeviceAligner(sync_base_ts=sync_base_ts)

    for p in npz_paths:
        d = load_npz(p)
        # 自动识别设备名
        name = p.stem.split("_")[0]  # emg / imu_left / imu_right
        if "emg" in p.stem.lower():
            name = "emg"
        elif "left" in p.stem.lower():
            name = "imu_left"
        elif "right" in p.stem.lower():
            name = "imu_right"
        else:
            name = p.stem.replace("_", "_")
        aligner.add_device(name, d)

    result = aligner.slice(t_start, t_end)

    if output:
        out_path = output
    else:
        out_path = Path(f"aligned_{int(t_start*1000)}_{int(t_end*1000)}.npz")

    # 合并为单个 npz
    save_dict = {}
    for name, sliced in result.items():
        for key, val in sliced.items():
            if isinstance(val, np.ndarray):
                save_dict[f"{name}_{key}"] = val
            elif isinstance(val, (int, float, str)):
                save_dict[f"{name}_{key}"] = np.array(val)

    save_dict["segment_start"] = np.float64(t_start)
    save_dict["segment_end"] = np.float64(t_end)
    save_dict["sync_base_ts"] = np.float64(sync_base_ts or 0.0)

    np.savez_compressed(out_path, **save_dict)
    print(f"\n[保存] {out_path}")


def cmd_align(npz_paths: list[Path], output: Path = None):
    """自动对齐所有设备."""
    print("\n[自动对齐] cross-correlation 找偏移")
    aligner = MultiDeviceAligner()

    for p in npz_paths:
        d = load_npz(p)
        name = p.stem
        aligner.add_device(name, d)

    # 识别 sEMG 主设备: 优先 key 含 "emg", 否则按内容(有 "data" 且无 "accel_xyz")
    emg_key = next((n for n in aligner.devices if "emg" in n.lower()), None)
    if emg_key is None:
        for n, dev in aligner.devices.items():
            npz = dev["npz"]
            if npz.get("data") is not None and npz.get("accel_xyz") is None:
                emg_key = n
                break
    if emg_key:
        print(f"[对齐] sEMG 主设备识别为: {emg_key}")
        aligner.auto_align(primary=emg_key)
    else:
        print("[WARN] 未识别到 sEMG 主设备(需要有 data 字段), 跳过 auto_align")

    # 只统计 secondary(非主设备)的对齐结果 — primary 是基准, offset 恒 0 不参与
    result = {name: {"offset_s": float(dev["offset_s"]),
                     "reliable": bool(dev.get("aligned", False))}
              for name, dev in aligner.devices.items() if name != emg_key}
    off_path = npz_paths[0].parent / "align_offsets.json"
    with open(off_path, "w", encoding="utf-8") as f:
        json.dump({"primary": emg_key, "devices": result}, f, ensure_ascii=False, indent=2)
    n_ok = sum(1 for d in result.values() if d["reliable"])
    n_fail = len(result) - n_ok
    print(f"[保存] 对齐结果 -> {off_path}  (可靠 {n_ok}/{len(result)})")
    if n_fail > 0:
        print(f"[警告] {n_fail} 个设备对齐失败(假offset), 不输出对齐数据 — 检查动作/电极后重采")
        return

    if output:
        np.savez_compressed(output, **{f"{k}_npz": v["npz"] for k, v in aligner.devices.items()})
        print(f"[保存] {output}")


def main():
    ap = argparse.ArgumentParser(
        description="sEMG + IMU 多设备数据对齐工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python align.py emg_session.npz imu_left.npz imu_right.npz
  python align.py session_dir/
  python align.py emg.npz --slice 10.0 25.5
  python align.py emg.npz imu_left.npz --auto-align
        """
    )
    ap.add_argument("files", nargs="*", help="npz 文件路径或目录")
    ap.add_argument("--config", type=str, help="从 sync.json 读取配置")
    ap.add_argument("--slice", nargs=2, type=float, metavar=("T_START", "T_END"),
                    help="按真实时间窗口切分 (秒)")
    ap.add_argument("--events", type=str, help="events.csv 路径, 按事件切分")
    ap.add_argument("--sync-base", type=float, help="全局 sync_base_ts (Unix 秒)")
    ap.add_argument("--auto-align", action="store_true",
                    help="用 cross-correlation 自动对齐")
    ap.add_argument("-o", "--output", type=str, help="输出文件路径")
    ap.add_argument("--info", action="store_true", help="只打印 npz 信息")
    args = ap.parse_args()

    # 解析文件列表
    paths = []
    for f in args.files:
        p = Path(f)
        if p.is_dir():
            paths.extend(p.glob("*.npz"))
        elif p.exists():
            paths.append(p)
        else:
            print(f"[WARN] 跳过不存在: {p}")

    if not paths:
        print("[ERR] 没有找到 npz 文件")
        ap.print_help()
        return 1

    paths = sorted(set(paths))
    # 过滤掉 orientation_trajectory 生成的姿态文件,只保留原始设备 npz
    paths = [p for p in paths if not p.stem.endswith("_orientation")]
    print(f"发现 {len(paths)} 个 npz 文件:")
    for p in paths:
        print(f"  {p.name}")

    # sync_base_ts
    sync_base = args.sync_base
    if args.config and not sync_base:
        cfg_path = Path(args.config)
        if cfg_path.is_dir():
            cfg_path = cfg_path / "sync.json"
        if cfg_path.exists():
            cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
            sync_base = cfg.get("global_sync_base_ts")
            print(f"\n从 sync.json 读取 sync_base_ts = {sync_base}")

    if args.info:
        cmd_info(paths)
        return 0

    if args.auto_align:
        cmd_align(paths, output=args.output)
        return 0

    if args.slice:
        t_start, t_end = args.slice
        cmd_slice(paths, t_start, t_end, output=args.output, sync_base_ts=sync_base)
        return 0

    # 默认: 打印信息
    cmd_info(paths)
    return 0


if __name__ == "__main__":
    sys.exit(main())
