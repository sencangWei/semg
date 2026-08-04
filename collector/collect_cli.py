"""LignoEMG-16CH 客户版命令行采集脚本 (CLI) - 双臂版.

不依赖 PyQt5 / torch, 客户电脑开箱即用.
- 自动枚举串口, 支持 1-2 台 LK-M1299 (默认双臂)
- 左/右臂各 16 通道, 合并为 32 通道统一输出
- 9 阶段按键 0-8 实时标注
- 实时打印采集进度与通道状态
- 一键保存为 CSV + NPZ + zip 三种格式

运行:  python collect_cli.py
退出:  Ctrl + C  (采集过程中按 ESC 停止并保存)
"""
import csv
import json
import os
import sys
import time
import datetime
import zipfile
import threading
from pathlib import Path

_HERE = Path(__file__).parent.resolve()
sys.path.insert(0, str(_HERE / "core"))

import serial
from serial.tools import list_ports

from ads129x_cmd import ADS129X_Cmd
from ads129x_data import ADS129X_Data

N_CHANNELS_PER_DEVICE = 16  # 每块 LK-M1299 的通道数
BAUDRATE = 2_000_000
OUTPUT_DIR = _HERE / "data" / "output"

GESTURES = [
    (0, "rest",      "休息 / 静息"),
    (1, "approach",  "1. 接近木块"),
    (2, "pregrasp",  "2. 准备抓握"),
    (3, "contact",   "3. 接触木块"),
    (4, "grasp",     "4. 抓握"),
    (5, "lift",      "5. 提举"),
    (6, "hold",      "6. 保持"),
    (7, "place",     "7. 放置"),
    (8, "release",   "8. 松开 / 复位"),
]

C = {
    "blue":   "\033[94m",
    "green":  "\033[92m",
    "yellow": "\033[93m",
    "red":    "\033[91m",
    "bold":   "\033[1m",
    "dim":    "\033[2m",
    "reset":  "\033[0m",
}


def color(s, c):
    return f"{C[c]}{s}{C['reset']}"


def banner():
    print(color("=" * 60, "blue"))
    print(color("  LignoEMG-16CH  双臂命令行采集工具  v2.0", "bold"))
    print(color("  支持 1-2 台设备: 左臂(L) + 右臂(R) = 32 通道", "dim"))
    print(color("=" * 60, "blue"))
    print()


def pick_ports() -> list:
    """
    让用户选择 1-2 个端口。
    返回 ["COM3"] (单臂) 或 ["COM3", "COM4"] (双臂)。
    """
    ports = sorted(list_ports.comports())
    if sys.platform != "win32":
        ports = [p for p in ports if not p.device.startswith("/dev/ttyS")]
    if not ports:
        sys.exit(color("[ERR] 未发现任何串口, 请检查 USB 连接", "red"))

    print(color("检测到的串口:", "yellow"))
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}  ({p.description})")

    if len(ports) == 1:
        chosen = [ports[0].device]
        print(color(f"自动选择唯一串口: {chosen[0]} (单臂模式)", "green"))
        return chosen

    if len(ports) == 2:
        chosen = [p.device for p in ports]
        print(color(f"自动检测到 2 个串口, 双臂模式: {chosen}", "green"))
        return chosen

    # >= 3 个端口, 让用户选择
    print()
    print(color("请选择左臂端口编号:", "bold"))
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}")
    left_idx = _read_index(len(ports)) - 1
    left_port = ports[left_idx].device

    print()
    print(color("请选择右臂端口编号 (选 0 = 单臂模式, 仅用左臂):", "bold"))
    for i, p in enumerate(ports, 1):
        marker = " <-- 同左臂" if p.device == left_port else ""
        print(f"  {i}. {p.device}{marker}")
    right_idx = _read_index(len(ports)) - 1
    if right_idx < 0:
        print(color("单臂模式", "yellow"))
        return [left_port]
    right_port = ports[right_idx].device
    if right_port == left_port:
        print(color("左右臂端口相同, 单臂模式", "yellow"))
        return [left_port]

    return [left_port, right_port]


def _read_index(max_val: int) -> int:
    """读取 1..max_val 的整数, 0 返回 0 (表示跳过)。"""
    while True:
        try:
            s = input(color(f"输入编号 [0-{max_val}]: ", "bold")).strip()
            idx = int(s)
            if 0 <= idx <= max_val:
                return idx
        except (ValueError, EOFError):
            pass
        print(color("输入无效, 请重试", "red"))


class Collector:
    """
    双臂采集器。
    支持 1 或 2 台 LK-M1299，数据合并为 (T, N) 统一存储。
    端口顺序: [左臂, 右臂]，左臂对应 emg_raw_data[0..15]，右臂对应 [16..31]。
    """

    def __init__(self, ports: list, customer: str, scene: str):
        self.ports = ports  # list of str, e.g. ["COM3"] 或 ["COM3", "COM4"]
        self.customer = customer or "default"
        self.scene = scene or "default"
        self.n_devices = len(ports)
        self.n_channels = self.n_devices * N_CHANNELS_PER_DEVICE
        self.arm_labels = ['L', 'R'][:self.n_devices]

        self.sers: list = []
        self.ads_list: list = []   # 每台设备一个 ADS129X_Data
        self.running = False
        self.collecting = False
        self.current_gesture_id = 0
        self.gesture_start_ts = None
        self.annotations = []
        self.lock = threading.Lock()
        self.reader_threads: list = []
        self.start_wall = None
        self.gesture_counts = {g[0]: 0 for g in GESTURES}

    def open(self):
        for i, port in enumerate(self.ports):
            arm = self.arm_labels[i]
            print(color(f"打开 {arm}臂 串口 {port} @ {BAUDRATE} ...", "yellow"))
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=BAUDRATE,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1,
                )
                if hasattr(ser, "set_buffer_size"):
                    ser.set_buffer_size(rx_size=102400, tx_size=65536)
                try:
                    ser.setRTS(False)
                    ser.setDTR(False)
                except (AttributeError, OSError):
                    pass
            except Exception as e:
                sys.exit(color(f"[ERR] 打开 {port} 失败: {e}", "red"))

            self.sers.append(ser)
            self.ads_list.append(ADS129X_Data())

        mode = "双臂" if self.n_devices == 2 else "左臂单边"
        print(color(f"[OK] {mode} {self.n_channels} 通道已就绪", "green"))

    def close(self):
        for ser in self.sers:
            if ser.is_open:
                ser.close()

    def send_all(self, cmd):
        for ser in self.sers:
            if ser.is_open:
                try:
                    ser.write(bytes(cmd))
                except Exception as e:
                    print(color(f"[WARN] 串口写入失败: {e}", "yellow"))

    def apply_sample_params(self):
        cmd = ADS129X_Cmd.set_sample_par_cmd("1000sps", "±375mV", 0x00FF)
        self.send_all(cmd)
        time.sleep(0.3)

    def start_collect(self):
        if not self.sers:
            print(color("[ERR] 串口未打开", "red"))
            return
        self.collecting = True
        self.start_wall = time.time()
        self.annotations = []
        for ads in self.ads_list:
            ads.clear()
        self.apply_sample_params()
        cmd = ADS129X_Cmd.start_collect_cmd()
        self.send_all(cmd)

        for i in range(self.n_devices):
            t = threading.Thread(target=self._reader, args=(i,), daemon=True)
            t.start()
            self.reader_threads.append(t)

        self.status_thread = threading.Thread(target=self._status, daemon=True)
        self.status_thread.start()

        mode = f"{self.n_devices} 台 {self.n_channels} 通道" if self.n_devices == 2 else "单臂 16 通道"
        print(color(f"[OK] 采集已启动, 按数字键 0-8 标注, q 停止并保存", "green"))

    def stop_collect(self):
        if not self.collecting:
            return
        self.collecting = False
        self.send_all(ADS129X_Cmd.stop_collect_cmd())
        time.sleep(0.2)
        if self.status_thread.is_alive():
            self.status_thread.join(timeout=1.0)
        self._print_summary()
        self.save()
        print(color("[OK] 采集已停止, 数据已保存", "green"))

    def _reader(self, device_idx: int):
        """读取指定设备的串口数据。"""
        ser = self.sers[device_idx]
        ads = self.ads_list[device_idx]
        while self.collecting:
            try:
                if ser.is_open:
                    avail = ser.in_waiting
                    if avail > 0:
                        data = ser.read(min(avail, 8192))
                        if data:
                            with self.lock:
                                ads.parse_data(data)
                    else:
                        time.sleep(0.005)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def _status(self):
        while self.collecting:
            time.sleep(3.0)
            with self.lock:
                totals = []
                for i, ads in enumerate(self.ads_list):
                    n = len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
                    totals.append(n)
                total_n = sum(totals)
            elapsed = time.time() - self.start_wall if self.start_wall else 0
            print(
                f"{color('[STATUS]', 'dim')}  "
                f"累计 {total_n} 样本 (L={totals[0]}" +
                (f" R={totals[1]}" if self.n_devices == 2 else "") +
                f")  已运行 {elapsed:.1f}s"
            )

    def set_gesture(self, gid: int):
        if gid == self.current_gesture_id:
            return
        now = time.time()
        if self.current_gesture_id is not None and self.gesture_start_ts is not None:
            self.annotations.append({
                "gesture_id": self.current_gesture_id,
                "gesture_name": next(g[1] for g in GESTURES if g[0] == self.current_gesture_id),
                "gesture_cn":   next(g[2] for g in GESTURES if g[0] == self.current_gesture_id),
                "start_ts":     self.gesture_start_ts,
                "end_ts":       now,
                "duration_s":   now - self.gesture_start_ts,
            })
            self.gesture_counts[self.current_gesture_id] += 1
        self.current_gesture_id = gid
        self.gesture_start_ts = now
        name_cn = next(g[2] for g in GESTURES if g[0] == gid)
        print(color(f"  >> 阶段 {gid}: {name_cn}", "yellow"))

    def _print_summary(self):
        print()
        print(color("=" * 50, "blue"))
        print(color("本次标注统计", "bold"))
        print(color("=" * 50, "blue"))
        for gid, en, cn in GESTURES:
            n = self.gesture_counts[gid]
            bar = "#" * min(40, n)
            print(f"  [{gid}] {cn:<14s} {n:>4d}  {bar}")
        print(color("=" * 50, "blue"))
        print()

    def save(self):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%mdd_%H%M%S")
        base = f"emg_{self.customer}_{self.scene}_{ts}"

        with self.lock:
            # 拉取各设备样本数（取最小值以保证时间对齐）
            ns = [len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
                  for ads in self.ads_list]
            n = min(ns) if ns else 0

        if n == 0:
            print(color("[WARN] 没有可保存的数据", "yellow"))
            return

        # 合并数据: (T, N) 左臂 + 右臂
        import numpy as np

        device_samples = []
        for ads in self.ads_list:
            ch_arrays = [np.array(ads.emg_raw_data[i][:n], dtype=np.float32)
                         for i in range(N_CHANNELS_PER_DEVICE)]
            device_samples.append(np.stack(ch_arrays, axis=1))  # (T, 16)
        data = np.concatenate(device_samples, axis=1)           # (T, 32) 或 (T, 16)

        # ---- NPZ (双臂合并) ----
        npz_path = OUTPUT_DIR / f"{base}.npz"
        np.savez(
            npz_path,
            data=data,
            sample_rate=1000,
            n_channels=self.n_channels,
            n_devices=self.n_devices,
            arm_labels=self.arm_labels,
            annotations=np.array(self.annotations, dtype=object),
            customer=self.customer,
            scene=self.scene,
        )

        # ---- CSV (左臂CH1-16, 右臂CH17-32) ----
        csv_path = OUTPUT_DIR / f"{base}.csv"
        channel_names = []
        for arm in self.arm_labels:
            for ch in range(1, N_CHANNELS_PER_DEVICE + 1):
                channel_names.append(f"CH{ch}_{arm}")

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([f"# customer={self.customer}", f"scene={self.scene}",
                        f"sample_rate=1000", f"n_channels={self.n_channels}",
                        f"n_devices={self.n_devices}"])
            w.writerow(["sample_idx"] + channel_names)
            for i in range(n):
                w.writerow([i] + [float(data[i, c]) for c in range(self.n_channels)])

        # ---- annotations JSON ----
        anno_path = OUTPUT_DIR / f"{base}_annotations.json"
        meta = {
            "customer": self.customer,
            "scene": self.scene,
            "sample_rate": 1000,
            "n_channels": self.n_channels,
            "n_devices": self.n_devices,
            "arm_labels": self.arm_labels,
            "channel_names": channel_names,
            "gesture_def": [{"id": g[0], "name": g[1], "name_cn": g[2]} for g in GESTURES],
            "gesture_counts": {str(k): int(v) for k, v in self.gesture_counts.items()},
            "annotations": self.annotations,
        }
        with open(anno_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        # ---- ZIP ----
        zip_path = OUTPUT_DIR / f"{base}.zip"
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.write(npz_path, arcname=Path(npz_path).name)
            zf.write(csv_path, arcname=Path(csv_path).name)
            zf.write(anno_path, arcname=Path(anno_path).name)
            readme = (
                f"LignoEMG-16CH 双臂采集数据\n"
                f"客户: {self.customer}\n"
                f"场景: {self.scene}\n"
                f"设备数: {self.n_devices}\n"
                f"通道数: {self.n_channels} (左臂 {N_CHANNELS_PER_DEVICE}"
                + (f", 右臂 {N_CHANNELS_PER_DEVICE}" if self.n_devices == 2 else "")
                + ")\n"
                f"采样率: 1000 Hz\n"
                f"样本数: {n}\n"
                f"采集时间: {datetime.datetime.now().isoformat()}\n\n"
                f"文件清单:\n"
                f"  - {Path(npz_path).name}  合并 EMG (numpy), shape=({n}, {self.n_channels})\n"
                f"  - {Path(csv_path).name}  CSV 表格\n"
                f"  - {Path(anno_path).name}  标注元数据 (JSON)\n"
                f"\n列顺序: 左臂 CH1-16, 右臂 CH1-16 (如为双臂模式)\n"
                f"         CH1-16 (如为单臂模式)\n"
            )
            zf.writestr("README.txt", readme)

        print()
        print(color("[保存清单]", "green"))
        print(f"  NPZ:  {npz_path}   shape=({n}, {self.n_channels})")
        print(f"  CSV:  {csv_path}")
        print(f"  JSON: {anno_path}")
        print(f"  ZIP:  {zip_path}   <-- 客户回传这一个文件即可")


def keyloop(collector: Collector):
    import msvcrt
    print()
    print(color("键位: 0-8 标注阶段  |  q 停止并保存  |  Ctrl+C 强制退出", "bold"))
    while True:
        try:
            ch = msvcrt.getch()
            if not ch:
                continue
            try:
                k = ch.decode("utf-8", errors="ignore")
            except Exception:
                k = ""
            if k == "q" or k == "Q":
                print(color("\n用户请求停止 ...", "yellow"))
                collector.stop_collect()
                return
            if k.isdigit():
                gid = int(k)
                if 0 <= gid <= 8:
                    collector.set_gesture(gid)
        except KeyboardInterrupt:
            print(color("\n捕获 Ctrl+C, 正在停止 ...", "yellow"))
            collector.stop_collect()
            return


def main():
    banner()
    if OUTPUT_DIR.exists() and not OUTPUT_DIR.is_dir():
        sys.exit(f"[ERR] 输出路径冲突: {OUTPUT_DIR}")

    customer = input(color("客户姓名 / 编号: ", "bold")).strip() or "default"
    scene = input(color("采集场景说明: ", "bold")).strip() or "default"

    ports = pick_ports()
    collector = Collector(ports=ports, customer=customer, scene=scene)

    try:
        collector.open()
    except Exception as e:
        sys.exit(color(f"[ERR] 打开串口失败: {e}", "red"))

    try:
        collector.start_collect()
        keyloop(collector)
    finally:
        collector.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(color("\n已退出", "yellow"))
