"""LignoEMG-16CH 纯 sEMG 采集器 v3.0 (客户交付版, 无动作标签).

设计原则:
  1. 客户可以自由执行任意任务 (分拣/抓握/放置...), 采集器不做任何动作分类.
  2. 客户也不需要按任何键, 全程连续记录 sEMG.
  3. 唯一需要的交互:
     - 启动采集 (回车)
     - 停止采集 (q 或 Ctrl+C)
     - 可选: 标记一个"事件点" (m + 回车), 用于将来和相机/IMU 对齐
  4. 时间戳是核心: 每个采样点都有精确 wall-clock 时间戳, 用于和 IMU/相机后处理对齐.
  5. 支持 TCP 远程触发: 相机/IMU 进程可以远程 start/stop/mark, 不依赖客户按任何键.

数据输出 (每个 session 一份):
  - emg_<session>.npz        包含:
      data         (T, N) float32 合并数据 (左臂 + 右臂)
      timestamps   (T,)   float64 wall-clock 时间戳 (Unix 秒, 秒级精度, 单调递增)
      sample_idx   (T,)   int32    从 0 开始的样本序号
      sample_rate  1000
      n_channels   16 或 32
      n_devices    1 或 2
      arm_labels   ["L"] 或 ["L", "R"]
      start_wall   采集开始 wall-clock
      stop_wall    采集停止 wall-clock
  - emg_<session>_events.csv  事件标记 (mark 或外部触发)
      seq, wall_ts, type, label, source
  - emg_<session>_meta.json   元数据
  - emg_<session>.zip         打包 (NPZ + CSV + JSON + README)

运行:
  python collect_pure.py                   # 默认交互式 (输入客户名 + 场景)
  python collect_pure.py --no-input       # 默认值, 不问任何问题 (相机触发模式)
  python collect_pure.py --tcp-port 8765   # 启动 TCP 触发服务器 (IMU/相机连接)

外部触发 (TCP):
  IMU/相机进程 → connect 127.0.0.1:8765 → send JSON line:
    {"action": "start", "session": "task_2026_07_07_01"}
    {"action": "mark",   "label": "pick_apple"}
    {"action": "stop"}
  服务端响应: JSON line {"ok": true, "msg": "started"} 等
"""

import argparse
import csv
import json
import os
import sys
import time
import socket
import threading
import zipfile
import datetime
from pathlib import Path

_HERE = Path(__file__).parent.resolve()
sys.path.insert(0, str(_HERE / "core"))

import numpy as np
import serial
from serial.tools import list_ports

from ads129x_cmd import ADS129X_Cmd
from ads129x_data import ADS129X_Data


# =============================================================================
# 基础常量
# =============================================================================
N_CHANNELS_PER_DEVICE = 16
SAMPLE_RATE_HZ = 1000
BAUDRATE = 2_000_000
OUTPUT_DIR = _HERE / "data" / "output"


# =============================================================================
# ANSI 颜色
# =============================================================================
C = {
    "blue":   "\033[94m",
    "green":  "\033[92m",
    "yellow": "\033[93m",
    "red":    "\033[91m",
    "cyan":   "\033[96m",
    "bold":   "\033[1m",
    "dim":    "\033[2m",
    "reset":  "\033[0m",
}


def color(s, c):
    return f"{C[c]}{s}{C['reset']}"


# =============================================================================
# 端口选择
# =============================================================================
def pick_ports() -> list:
    """选择 1-2 个串口. 数量 = 设备数."""
    ports = sorted(list_ports.comports())
    # 跳过蓝牙虚拟串口(BTHENUM), 它们不是 sEMG
    bt = [p for p in ports if "BTHENUM" in (p.hwid or "")]
    if bt:
        print(color(f"(已跳过 {len(bt)} 个蓝牙串口: {', '.join(p.device for p in bt)})", "dim"))
    ports = [p for p in ports if "BTHENUM" not in (p.hwid or "")]
    if sys.platform != "win32":
        ports = [p for p in ports if not p.device.startswith("/dev/ttyS")]
    if not ports:
        sys.exit(color("[ERR] 未发现任何串口 (已排除蓝牙). 请检查 sEMG 接收器 USB 是否插好", "red"))

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

    print()
    print(color("请选择左臂端口编号 (输入 0 = 仅用左臂):", "bold"))
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}")
    left_idx = _read_index(len(ports)) - 1

    print()
    print(color("请选择右臂端口编号 (输入 0 = 单臂):", "bold"))
    for i, p in enumerate(ports, 1):
        marker = " <-- 同左臂" if p.device == ports[left_idx].device else ""
        print(f"  {i}. {p.device}{marker}")
    right_idx = _read_index(len(ports)) - 1

    if right_idx < 0 or right_idx == left_idx:
        print(color("单臂模式", "yellow"))
        return [ports[left_idx].device]
    return [ports[left_idx].device, ports[right_idx].device]


def auto_pick_ports() -> list:
    """非交互式自动选择串口. 默认只选 1 个,适配单串口双臂(16CH)模式."""
    ports = sorted(list_ports.comports())
    # 跳过蓝牙虚拟串口(BTHENUM), 它们不是 sEMG
    bt = [p for p in ports if "BTHENUM" in (p.hwid or "")]
    if bt:
        print(color(f"(已跳过 {len(bt)} 个蓝牙串口: {', '.join(p.device for p in bt)})", "dim"))
    ports = [p for p in ports if "BTHENUM" not in (p.hwid or "")]
    if sys.platform != "win32":
        ports = [p for p in ports if not p.device.startswith("/dev/ttyS")]
    if not ports:
        sys.exit(color("[ERR] 未发现任何串口 (已排除蓝牙). 请检查 sEMG 接收器 USB 是否插好", "red"))

    print(color("检测到的串口:", "yellow"))
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}  ({p.description})")

    chosen = [ports[0].device]
    if len(ports) > 1:
        print(color(f"[WARN] 检测到多个串口,非交互模式默认只选第 1 个: {chosen[0]}", "yellow"))
        print(color("       当前为单串口双臂模式(16 通道: ch1-8 右手, ch9-16 左手)", "dim"))
        print(color("       若你是双设备(32CH)模式,请在 config.json 的 semg.ports 中指定两个口", "dim"))
    else:
        print(color(f"自动选择唯一串口: {chosen[0]} (单串口双臂 16 通道)", "green"))
    return chosen


def _read_index(max_val: int) -> int:
    while True:
        try:
            s = input(color(f"输入编号 [0-{max_val}]: ", "bold")).strip()
            idx = int(s)
            if 0 <= idx <= max_val:
                return idx
        except (ValueError, EOFError):
            pass
        print(color("输入无效, 请重试", "red"))


# =============================================================================
# 采集核心: 1-2 台 LK-M1299 合并为 (T, N), 同时记录时间戳
# =============================================================================
class PureCollector:
    """
    纯 sEMG 采集器. 不做任何动作分类/标注.
    唯一事件接口:
      - mark_event(label, source)  记录一个时间戳锚点 (用于和 IMU/相机对齐)
      - start() / stop()           启动/停止
      - save(path)                 保存为 NPZ + JSON + CSV
    """

    def __init__(self, ports: list, session: str = None,
                 tcp_port: int = None):
        self.ports = ports
        self.n_devices = len(ports)
        self.n_channels = self.n_devices * N_CHANNELS_PER_DEVICE
        self.arm_labels = ['L', 'R'][:self.n_devices]
        self.session = session or datetime.datetime.now().strftime("emg_%Y%m%d_%H%M%S")

        self.sers: list = []
        self.ads_list: list = []

        # 状态
        self.lock = threading.Lock()
        self.collecting = False
        self.start_wall = None       # float, Unix 秒
        self.stop_wall = None
        self.sync_base_ts = None     # float, Unix 秒, 所有设备的同步基准

        # 时间戳队列 (与 emg_raw_data[0] 长度同步)
        self.timestamps: list = []   # 每个采样点的 wall-clock 时间戳

        # 事件标记 (mark 键或外部触发)
        self.events: list = []       # [{seq, ts, type, label, source}]

        # 后台线程
        self.reader_threads: list = []
        self.status_thread: list = []

        # TCP 服务器
        self.tcp_port = tcp_port
        self.tcp_server = None
        self.tcp_clients: list = []  # list of (sock, addr)
        self.tcp_thread = None

    # ---------------- 设备 IO ----------------
    def open(self):
        for i, port in enumerate(self.ports):
            if self.n_devices == 1 and self.n_channels == 16:
                label = "sEMG"
            else:
                label = f"{self.arm_labels[i]}臂"
            print(color(f"打开 {label} 串口 {port} @ {BAUDRATE} ...", "yellow"))
            try:
                ser = serial.Serial(
                    port=port, baudrate=BAUDRATE,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=0.1,
                )
                if hasattr(ser, "set_buffer_size"):
                    ser.set_buffer_size(rx_size=102400, tx_size=65536)
                try:
                    ser.setRTS(False)
                    ser.setDTR(False)
                except (AttributeError, OSError):
                    pass
            except Exception as e:
                self.close()
                sys.exit(color(f"[ERR] 打开 {port} 失败: {e}", "red"))
            self.sers.append(ser)
            self.ads_list.append(ADS129X_Data())

        if self.n_devices == 1 and self.n_channels == 16:
            mode = "单串口双臂"
            note = "(ch1-8 右手, ch9-16 左手)"
        elif self.n_devices == 2:
            mode = "双臂（双串口）"
            note = ""
        else:
            mode = "单臂"
            note = ""
        print(color(f"[OK] {mode} {self.n_channels} 通道已就绪 {note}", "green"))

    def _channel_arm_labels(self) -> list:
        """每个通道对应的臂标签. 单串口 16 通道时 ch1-8 为右手, ch9-16 为左手."""
        if self.n_devices == 1 and self.n_channels == 16:
            return ['R'] * 8 + ['L'] * 8
        # 双串口: 按设备顺序(默认第 1 个设备 = 左臂, 第 2 个 = 右臂)
        labels = []
        for arm in self.arm_labels:
            labels.extend([arm] * N_CHANNELS_PER_DEVICE)
        return labels

    def close(self):
        for ser in self.sers:
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass

    def _send_all(self, cmd):
        for ser in self.sers:
            try:
                if ser.is_open:
                    ser.write(bytes(cmd))
            except Exception as e:
                print(color(f"[WARN] 串口写入失败: {e}", "yellow"))

    def _apply_sample_params(self):
        # 通道 mask: 0xFFFF = 开全部 16 通道(双臂); 0x00FF = 只开前 8(单臂)
        self._send_all(ADS129X_Cmd.set_sample_par_cmd("1000sps", "±375mV", 0xFFFF))
        time.sleep(0.3)

    # ---------------- 采集控制 ----------------
    def start(self, countdown_s: float = 0, sync_base_ts: float = None):
        """
        启动采集. 可选预热倒计时.

        Args:
            countdown_s: 预热秒数.
                        启动后先等 N 秒 (客户准备时间),
                        倒计时结束瞬间记录 sync_base_ts 作为所有设备的同步基准.
                        这 N 秒内 sEMG 已经流入缓冲区, 倒计时结束后才开始正式记录.
            sync_base_ts: 外部传入的全局同步基准 (例如 start_all.py 统一指定).
                          为 None 时使用当前时间.
        """
        if not self.sers:
            print(color("[ERR] 串口未打开", "red"))
            return False
        if self.collecting:
            return True

        # 配置采样参数 (设备已经准备好)
        self._apply_sample_params()

        # 正式标记开始 (必须在启动 reader 之前置 collecting=True:
        # reader 循环首句就是 "if not self.collecting: break",
        # 若 collecting 还是 False, reader 线程一启动就立刻退出 → 收不到数据)
        with self.lock:
            self.collecting = True
            self.start_wall = time.time()
            self.stop_wall = None
            self.sync_base_ts = sync_base_ts if sync_base_ts is not None else self.start_wall
            self.timestamps = []
            self.events = []
            for ads in self.ads_list:
                ads.clear()

        # 启动数据读取线程 (在倒计时期间就开始接收数据)
        for i in range(self.n_devices):
            t = threading.Thread(target=self._reader, args=(i,), daemon=True)
            t.start()
            self.reader_threads.append(t)

        # 时间戳线程
        ts_t = threading.Thread(target=self._timestamp_pacer, daemon=True)
        ts_t.start()
        self.reader_threads.append(ts_t)

        # 预热倒计时
        if countdown_s > 0:
            self._apply_sample_params()
            self._send_all(ADS129X_Cmd.start_collect_cmd())
            print(color(f"\n[倒计时] 等待 {countdown_s:.0f}s 客户准备, 期间 sEMG 已在缓冲 ...", "yellow"))
            for i in range(int(countdown_s), 0, -1):
                print(color(f"  {i} ...", "yellow"))
                time.sleep(1.0)
            print(color(f"  开始! sync_base_ts = {time.time():.6f}", "green"))

        if countdown_s <= 0:
            # 立即开始: 发硬件 start 命令
            self._apply_sample_params()
            self._send_all(ADS129X_Cmd.start_collect_cmd())

        # TCP 服务器
        if self.tcp_port:
            self._start_tcp_server()

        # 状态打印线程
        st = threading.Thread(target=self._status_printer, daemon=True)
        st.start()
        self.status_thread.append(st)

        mode = f"{self.n_channels} 通道"
        print(color(f"[OK] 采集已启动 ({mode} @ {SAMPLE_RATE_HZ}Hz)", "green"))
        print(color("      sync_base_ts = " + f"{self.sync_base_ts:.6f} (Unix 秒, 用于跨设备对齐)", "dim"))
        print(color("      客户自由操作, 不需要按任何键", "dim"))
        if not self.tcp_port:
            print(color("      终端命令: m <label> 标记事件 / q 停止", "dim"))
        else:
            print(color(f"      TCP 触发服务器: 127.0.0.1:{self.tcp_port}", "dim"))
        return True

    def stop(self):
        if not self.collecting:
            return
        with self.lock:
            self.collecting = False
            self.stop_wall = time.time()
        self._send_all(ADS129X_Cmd.stop_collect_cmd())
        time.sleep(0.2)

        # 关闭 TCP
        if self.tcp_server:
            try:
                self.tcp_server.shutdown(socket.SHUT_RDWR)
                self.tcp_server.close()
            except Exception:
                pass
            self.tcp_server = None

        # 等待线程退出
        for t in self.reader_threads:
            if t.is_alive():
                t.join(timeout=1.0)
        for t in self.status_thread:
            if t.is_alive():
                t.join(timeout=0.5)
        self.reader_threads.clear()
        self.status_thread.clear()

        # 关闭串口
        self.close()

        n_samples = self._current_n_samples()
        elapsed = (self.stop_wall - self.start_wall) if self.start_wall and self.stop_wall else 0
        print(color(f"[OK] 采集已停止: {n_samples} 样本, {elapsed:.1f}s, {len(self.events)} 事件", "green"))

    # ---------------- 读串口 ----------------
    def _reader(self, device_idx: int):
        ser = self.sers[device_idx]
        ads = self.ads_list[device_idx]
        while True:
            with self.lock:
                if not self.collecting:
                    break
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

    def _timestamp_pacer(self):
        """
        每 ~10ms 把这段时间内新增的样本打上当前时间戳.
        精度: 约 ±5ms, 适合和 IMU/相机后处理做 cross-correlation 对齐.
        """
        last_n_per_device = [0] * self.n_devices
        while True:
            with self.lock:
                if not self.collecting:
                    break
                now = time.time()
                # 找所有设备中样本最少的 (因为两端时钟漂移, 最小值 = 安全同步点)
                cur_ns = [len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
                          for ads in self.ads_list]
                delta_per_dev = [cur_ns[i] - last_n_per_device[i] for i in range(self.n_devices)]
                # 增量时间戳 = 这段时间内所有设备的新增样本数 = 平均值
                avg_delta = sum(delta_per_dev) // self.n_devices if self.n_devices else 0
                # 给这 avg_delta 个样本打时间戳: 均匀分布到 (now - avg_delta/1000, now]
                if avg_delta > 0:
                    ts_block = np.linspace(
                        now - avg_delta / SAMPLE_RATE_HZ,
                        now,
                        avg_delta,
                        endpoint=False
                    )
                    self.timestamps.extend(ts_block.tolist())
                last_n_per_device = cur_ns
            time.sleep(0.01)

    def _current_n_samples(self) -> int:
        with self.lock:
            ns = [len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
                  for ads in self.ads_list]
            return min(ns) if ns else 0

    # ---------------- 状态打印 ----------------
    def _status_printer(self):
        while True:
            with self.lock:
                if not self.collecting:
                    break
            time.sleep(3.0)
            n = self._current_n_samples()
            elapsed = time.time() - self.start_wall if self.start_wall else 0
            rate = n / elapsed if elapsed > 0 else 0
            print(
                f"{color('[STATUS]', 'dim')}  "
                f"累计 {n} 样本  "
                f"已运行 {elapsed:.1f}s  "
                f"实测速率 {rate:.0f} Hz  "
                f"事件 {len(self.events)}"
            )

    # ---------------- 事件标记 ----------------
    def mark_event(self, label: str = "", source: str = "cli"):
        """
        记录一个事件标记. 用于和 IMU/相机对齐.
        事件会写进 events.csv, 每行包含 (seq, ts, type, label, source).
        """
        # 先取锁后做所有事, 不要在持锁状态下调用 _current_n_samples()
        # 否则会与 reader/timestamp 线程死锁
        with self.lock:
            if not self.collecting:
                return False
            seq = len(self.events) + 1
            ts = time.time()
            n = min((len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0)
                    for ads in self.ads_list)
            evt = {
                "seq": seq,
                "wall_ts": ts,
                "sample_idx": n,
                "type": "mark",
                "label": label or f"event_{seq}",
                "source": source,
            }
            self.events.append(evt)
        print(color(
            f"  >> 事件 #{seq}  sample_idx={n}  ts={ts:.6f}  "
            f"label={evt['label']}  source={source}",
            "cyan"
        ))
        return True

    # ---------------- TCP 触发服务器 ----------------
    def _start_tcp_server(self):
        if self.tcp_server is not None:
            return
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.tcp_server.bind(("127.0.0.1", self.tcp_port))
        except OSError as e:
            print(color(f"[WARN] TCP 端口 {self.tcp_port} 被占用, 跳过外部触发 ({e})", "yellow"))
            self.tcp_server = None
            return
        self.tcp_server.listen(2)
        self.tcp_server.settimeout(0.5)
        self.tcp_thread = threading.Thread(target=self._tcp_accept_loop, daemon=True)
        self.tcp_thread.start()
        print(color(f"[OK] TCP 触发服务器已启动: 127.0.0.1:{self.tcp_port}", "green"))

    def _tcp_accept_loop(self):
        # TCP accept 循环, 持续运行直到 stop() 关闭 self.tcp_server
        # 不要在 collecting=False 时退出, 因为 TCP 启动可能早于 start()
        while True:
            if self.tcp_server is None:
                return
            try:
                sock, addr = self.tcp_server.accept()
            except socket.timeout:
                continue
            except OSError:
                # self.tcp_server 被 stop() 关闭
                return
            self.tcp_clients.append(sock)
            t = threading.Thread(target=self._tcp_handle, args=(sock, addr), daemon=True)
            t.start()

    def _tcp_handle(self, sock: socket.socket, addr):
        sock.settimeout(None)
        buf = b""
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    resp = self._tcp_dispatch(line.decode("utf-8", errors="ignore"))
                    try:
                        sock.sendall((json.dumps(resp) + "\n").encode("utf-8"))
                    except Exception:
                        return
        except Exception:
            pass
        finally:
            try:
                sock.close()
            except Exception:
                pass
            if sock in self.tcp_clients:
                self.tcp_clients.remove(sock)

    def _tcp_dispatch(self, line: str) -> dict:
        """处理外部触发的 JSON 命令."""
        try:
            req = json.loads(line)
        except Exception as e:
            return {"ok": False, "err": f"json parse: {e}"}
        action = req.get("action", "").lower()
        session = req.get("session", self.session)
        countdown = req.get("countdown", 0)
        if action == "start":
            if not self.collecting:
                ok = self.start(countdown_s=float(countdown),
                                sync_base_ts=req.get("sync_base_ts"))
            else:
                ok = True
            return {"ok": ok, "state": "collecting" if self.collecting else "stopped",
                    "sync_base_ts": self.sync_base_ts,
                    "start_wall": self.start_wall}
        if action == "stop":
            self.stop()
            return {"ok": True, "state": "stopped",
                    "sync_base_ts": self.sync_base_ts,
                    "start_wall": self.start_wall,
                    "n_samples": self._current_n_samples(),
                    "n_events": len(self.events),
                    "duration_s": (self.stop_wall - self.start_wall) if (self.stop_wall and self.start_wall) else 0}
        if action == "mark":
            label = req.get("label", "")
            source = req.get("source", "tcp")
            ok = self.mark_event(label=label, source=source)
            return {"ok": ok, "event_seq": len(self.events),
                    "sync_base_ts": self.sync_base_ts}
        if action == "ping":
            return {"ok": True, "state": "collecting" if self.collecting else "stopped",
                    "n_samples": self._current_n_samples(),
                    "n_events": len(self.events),
                    "sync_base_ts": self.sync_base_ts,
                    "start_wall": self.start_wall,
                    "elapsed_s": time.time() - self.start_wall if self.start_wall else 0}
        return {"ok": False, "err": f"unknown action: {action}"}

    # ---------------- 保存 ----------------
    def save(self, out_dir: Path = None):
        out_dir = Path(out_dir) if out_dir else OUTPUT_DIR
        out_dir.mkdir(parents=True, exist_ok=True)

        with self.lock:
            ns = [len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
                  for ads in self.ads_list]
            n = min(ns) if ns else 0

        if n == 0:
            print(color("[WARN] 没有可保存的数据", "yellow"))
            return None

        # 合并数据: (T, N) 左臂 + 右臂
        device_samples = []
        for ads in self.ads_list:
            ch_arrays = [np.array(ads.emg_raw_data[i][:n], dtype=np.float32)
                         for i in range(N_CHANNELS_PER_DEVICE)]
            device_samples.append(np.stack(ch_arrays, axis=1))   # (T, 16)
        data = np.concatenate(device_samples, axis=1)            # (T, 32) 或 (T, 16)

        # 截取时间戳到 n 长度
        ts = np.array(self.timestamps[:n], dtype=np.float64)
        if len(ts) < n:
            # 补齐 (启动瞬间可能差几个样本)
            pad = np.linspace(ts[-1] if len(ts) else self.start_wall,
                              self.stop_wall or time.time(),
                              n - len(ts) + 1, endpoint=True)[1:]
            ts = np.concatenate([ts, pad])

        sample_idx = np.arange(n, dtype=np.int32)
        channel_arm_labels = self._channel_arm_labels()
        channel_names = [f"CH{c+1}_{arm}"
                         for c, arm in enumerate(channel_arm_labels)]

        base = f"emg_{self.session}"
        npz_path = out_dir / f"{base}.npz"

        # 相对同步基准的时间 (秒) - 对齐 IMU/相机时用这个最方便
        sync_offset = ts - self.sync_base_ts  # shape (T,)
        np.savez_compressed(
            npz_path,
            data=data,
            timestamps=ts,             # 真实 Unix 秒
            elapsed_since_sync=sync_offset,  # 相对 sync_base_ts 的秒数 (核心对齐字段)
            sample_idx=sample_idx,
            sample_rate=SAMPLE_RATE_HZ,
            n_channels=self.n_channels,
            n_devices=self.n_devices,
            arm_labels=np.array(self.arm_labels),
            channel_arm_labels=np.array(channel_arm_labels),
            channel_names=np.array(channel_names),
            start_wall=np.float64(self.start_wall or 0.0),
            stop_wall=np.float64(self.stop_wall or 0.0),
            sync_base_ts=np.float64(self.sync_base_ts or 0.0),
            events=np.array(self.events, dtype=object),
        )

        # 事件 CSV
        csv_path = out_dir / f"{base}_events.csv"
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["seq", "wall_ts", "sample_idx", "type", "label", "source"])
            for e in self.events:
                w.writerow([e["seq"], f"{e['wall_ts']:.6f}", e["sample_idx"],
                            e["type"], e["label"], e["source"]])

        # 元数据 JSON
        meta_path = out_dir / f"{base}_meta.json"
        meta = {
            "session": self.session,
            "customer": getattr(self, "_customer", ""),
            "scene": getattr(self, "_scene", ""),
            "n_devices": self.n_devices,
            "n_channels": self.n_channels,
            "arm_labels": self.arm_labels,
            "channel_arm_labels": channel_arm_labels,
            "channel_names": channel_names,
            "sample_rate_hz": SAMPLE_RATE_HZ,
            "n_samples": int(n),
            "start_wall": self.start_wall,
            "stop_wall": self.stop_wall,
            "duration_s": (self.stop_wall - self.start_wall) if (self.stop_wall and self.start_wall) else 0,
            "sync_base_ts": self.sync_base_ts,   # 同步基准: 所有设备共用此真实时间
            "n_events": len(self.events),
            "events": self.events,
        }
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        # 打包
        zip_path = out_dir / f"{base}.zip"
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.write(npz_path, arcname=npz_path.name)
            zf.write(csv_path, arcname=csv_path.name)
            zf.write(meta_path, arcname=meta_path.name)
            readme = (
                f"LignoEMG-16CH 纯 sEMG 采集数据 (无动作标签)\n"
                f"Session: {self.session}\n"
                f"客户: {getattr(self, '_customer', '')}\n"
                f"场景: {getattr(self, '_scene', '')}\n"
                f"设备数: {self.n_devices} ({', '.join(self.arm_labels)}臂)\n"
                f"通道数: {self.n_channels}  采样率: {SAMPLE_RATE_HZ} Hz\n"
                f"样本数: {n}  时长: {meta['duration_s']:.2f}s\n"
                f"事件数: {len(self.events)}\n"
                f"开始: {datetime.datetime.fromtimestamp(self.start_wall).isoformat()}\n"
                f"停止: {datetime.datetime.fromtimestamp(self.stop_wall).isoformat() if self.stop_wall else 'N/A'}\n\n"
                f"文件清单:\n"
                f"  - {npz_path.name}     合并 EMG (numpy)\n"
                f"                         data (T, N) float32\n"
                f"                         timestamps (T,) float64, Unix 秒\n"
                f"                         sample_idx (T,) int32\n"
                f"  - {csv_path.name}   事件标记 (mark)\n"
                f"  - {meta_path.name}  元数据 (JSON)\n"
                f"\n用途说明:\n"
                f"  - 客户执行任意任务, 采集器不分类不标注.\n"
                f"  - 时间戳用于和 IMU/相机后处理对齐.\n"
                f"  - 若训练需要动作标签, 用相机/IMU 后处理自动标注.\n"
            )
            zf.writestr("README.txt", readme)

        print()
        print(color("[保存清单]", "green"))
        print(f"  NPZ:   {npz_path}   shape=({n}, {self.n_channels})")
        print(f"  CSV:   {csv_path}  ({len(self.events)} 事件)")
        print(f"  JSON:  {meta_path}")
        print(f"  ZIP:   {zip_path}  <-- 客户回传此文件即可")
        return zip_path


# =============================================================================
# 键盘输入 (跨平台)
# =============================================================================
def keyloop(collector: PureCollector, no_input: bool = False):
    """
    跨平台键盘监听:
      m + 回车        标记一个事件 (可选填 label)
      q + 回车 或 Ctrl+C  停止采集
    no_input=True 时不读键盘 (纯 TCP 触发模式).
    """
    if no_input:
        print(color("[NO-INPUT] 不读键盘, 等待 TCP 触发或 Ctrl+C", "yellow"))
        try:
            while collector.collecting:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        collector.stop()
        return

    print()
    print(color("终端命令:", "bold"))
    print(color("  m <label>      标记事件 (例: m pick_apple)", "dim"))
    print(color("  q              停止采集", "dim"))
    print(color("  Ctrl+C         强制停止", "dim"))
    print()

    try:
        while True:
            try:
                line = input(color("> ", "bold"))
            except EOFError:
                break
            line = line.strip()
            if not line:
                continue
            parts = line.split(maxsplit=1)
            cmd = parts[0].lower()
            if cmd == "q":
                break
            if cmd == "m":
                label = parts[1] if len(parts) > 1 else ""
                collector.mark_event(label=label, source="cli")
            else:
                print(color(f"未知命令: {cmd} (输入 m 或 q)", "yellow"))
    except KeyboardInterrupt:
        pass

    collector.stop()


# =============================================================================
# 入口
# =============================================================================
def banner():
    print(color("=" * 60, "blue"))
    print(color("  LignoEMG-16CH  纯 sEMG 采集器 v3.0", "bold"))
    print(color("  客户自由动作 · 无标签 · 精确时间戳 · TCP 触发", "dim"))
    print(color("=" * 60, "blue"))
    print()


def main():
    ap = argparse.ArgumentParser(description="LignoEMG-16CH 纯 sEMG 采集器")
    ap.add_argument("--no-input", action="store_true",
                    help="不读键盘, 仅 TCP 触发 / Ctrl+C")
    ap.add_argument("--tcp-port", type=int, default=None,
                    help="启动 TCP 触发服务器 (例: --tcp-port 8765)")
    ap.add_argument("--session", type=str, default=None,
                    help="session 名 (默认时间戳)")
    ap.add_argument("--customer", type=str, default="",
                    help="客户名 (元数据)")
    ap.add_argument("--scene", type=str, default="",
                    help="场景说明 (元数据)")
    ap.add_argument("--countdown", type=float, default=0,
                    help="预热倒计时秒数 (等待客户准备, 默认0)")
    ap.add_argument("--ports", type=str, default=None,
                    help="逗号分隔串口列表, 例如 COM3,COM4; 不指定则自动检测")
    ap.add_argument("--output-dir", type=str, default=None,
                    help="输出目录 (默认 ./data/output)")
    ap.add_argument("--wait-for-start", action="store_true",
                    help="TCP 模式下等待外部 start 命令再开始采集")
    args = ap.parse_args()

    banner()

    if not args.no_input:
        if not args.customer:
            args.customer = input(color("客户姓名 / 编号 [回车=default]: ", "bold")).strip() or "default"
        if not args.scene:
            args.scene = input(color("采集场景说明 [回车=default]: ", "bold")).strip() or "default"

    if args.ports:
        ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    elif args.no_input:
        ports = auto_pick_ports()
    else:
        ports = pick_ports()

    out_dir = Path(args.output_dir) if args.output_dir else OUTPUT_DIR

    collector = PureCollector(ports=ports, session=args.session,
                              tcp_port=args.tcp_port)
    collector._customer = args.customer
    collector._scene = args.scene

    try:
        collector.open()
    except SystemExit:
        raise
    except Exception as e:
        sys.exit(color(f"[ERR] 打开串口失败: {e}", "red"))

    if args.wait_for_start:
        if not args.tcp_port:
            print(color("[ERR] --wait-for-start 必须配合 --tcp-port 使用", "red"))
            sys.exit(1)
        collector._start_tcp_server()
        print(color("[NO-INPUT] 等待 start_all.py 发送 start 命令...", "yellow"))
        try:
            while not collector.collecting:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(color("\n[EXIT] 未等到 start 命令,退出", "yellow"))
            collector.stop()
            return
        keyloop(collector, no_input=True)
    else:
        try:
            collector.start(countdown_s=args.countdown)
            keyloop(collector, no_input=args.no_input)
        finally:
            try:
                collector.save(out_dir=out_dir)
            except Exception as e:
                print(color(f"[ERR] 保存失败: {e}", "red"))
                import traceback
                traceback.print_exc()
        return

    # --wait-for-start 路径的保存
    try:
        collector.save(out_dir=out_dir)
    except Exception as e:
        print(color(f"[ERR] 保存失败: {e}", "red"))
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(color("\n已退出", "yellow"))
