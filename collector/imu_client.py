"""IMU 采集器客户端 - 接收 ESP32+BNO085 WiFi 推送的数据.

运行在 PC 端，通过 WiFi/TCP 连接 ESP32-S3 (作为 TCP 服务器).
BNO085 传感器数据 (四元数/加速度/陀螺仪) 通过 SHTP 协议读出,
ESP32 通过 WiFi 将数据 TCP 推送到本客户端, 客户端记录 wall-clock 时间戳.

用法:
  python imu_client.py --addr 192.168.4.1 --port 8766
  python imu_client.py --addr 192.168.4.1 --port 8766 --session my_session

TCP 命令接口 (接受 start_all.py 的控制):
  {"action": "start"}
  {"action": "stop"}
  {"action": "mark", "label": "pick_apple"}
  {"action": "ping"}

数据输出:
  imu_left_<session>.npz   (双腕各一个文件)
  imu_left_<session>_events.csv
  imu_left_<session>_meta.json

数据字段:
  timestamps   (T,)   float64, Unix 秒 (wall-clock)
  sensor_time_us (T,) uint64, ESP32 micros() 时钟
  quat_wxyz   (T, 4) float32, 四元数 (w, x, y, z)
  quat_game_wxyz (T, 4) float32, 6轴 Game Rotation Vector
  accel_xyz   (T, 3) float32, 加速度 (m/s^2)
  gyro_xyz    (T, 3) float32, 陀螺仪 (rad/s)
  mag_xyz     (T, 3) float32, 地磁场 (a.u.)
  sample_rate_approx  float32, 近似采样率
"""

import argparse
import csv
import json
import socket
import struct
import sys
import threading
import time
import zipfile
import datetime
from pathlib import Path
from collections import deque

import numpy as np

try:
    from .orientation_trajectory import build_orientation_trajectory, save_trajectory
except ImportError:
    from orientation_trajectory import build_orientation_trajectory, save_trajectory


# =============================================================================
# 常量
# =============================================================================
DEFAULT_PORT = 8766
DEFAULT_ADDR = "192.168.4.1"
OUTPUT_DIR = Path(__file__).parent / "data" / "output"
SAMPLE_RATE_APPROX = 200  # Hz, BNO085 当前报告率 (与 firmware_imu/src/main.cpp 的 SEND_HZ 一致)
ORIENTATION_AXIS_MAPS = {
    "left": "+x,-y,-z",   # axis_calib_left_03 标定结果：+x=旋前, +y=向上/屈, +z=向右
    "right": "+x,+y,+z",  # 右手已标定确认
}
ORIENTATION_BASELINE_S = 2.0

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
# BNO08x SHTP 协议解析 (从 ESP32 TCP 流接收)
# =============================================================================

class BNO08xParser:
    """
    解析 ESP32 推送的 BNO085 SHTP 数据包.
    ESP32 固件 (Arduino/PlatformIO) 将 SHTP 报告编码为二进制帧:

    帧格式 (每帧一行, 二进制或 JSON):
      方案 A (推荐): 固定 52 字节二进制
        struct.pack('I7f',
          timestamp_us,     # uint32, 微秒
          qw, qx, qy, qz,  # float, 四元数
          ax, ay, az,       # float, 加速度 m/s^2
          [gx, gy, gz])     # float, 陀螺仪 rad/s

      方案 B: JSON 文本行
        {"t": 1234567, "q": [w,x,y,z], "a": [x,y,z], "g": [x,y,z]}
    """

    def __init__(self):
        self.buf = b""

    def feed(self, data: bytes):
        self.buf += data

    def parse_lines(self) -> list:
        """从 buf 中提取所有完整的帧, 返回 list of dict."""
        frames = []
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            frame = self._parse_one(line)
            if frame:
                frames.append(frame)
        return frames

    def _parse_one(self, line: bytes) -> dict:
        if line.startswith(b"{"):
            # JSON 格式
            try:
                obj = json.loads(line.decode("utf-8", errors="ignore"))
                return self._normalize_json(obj)
            except Exception:
                return None
        else:
            # 二进制格式 (52 字节: 4B timestamp + 7×4B floats = 32B)
            # 或 (60 字节: 4B + 10×4B = 44B) 带 gyro
            if len(line) >= 32:
                try:
                    ts_us, qw, qx, qy, qz, ax, ay = struct.unpack("<I7f", line[:32])
                    result = {
                        "timestamp_us": ts_us,
                        "quat": [qw, qx, qy, qz],
                        "accel": [ax, ay, 0.0],
                        "gyro": [0.0, 0.0, 0.0],
                    }
                    if len(line) >= 44:
                        gx, gy, gz = struct.unpack("<3f", line[32:44])
                        result["gyro"] = [gx, gy, gz]
                    return result
                except Exception:
                    pass
            return None

    def _normalize_json(self, obj: dict) -> dict:
        """把各种 JSON 格式统一成 {timestamp_us, quat, accel, gyro, lacc}."""
        ts = obj.get("t") or obj.get("ts") or obj.get("timestamp_us", 0)
        q = obj.get("q") or obj.get("quat") or [1.0, 0.0, 0.0, 0.0]
        q6 = obj.get("q6") or obj.get("quat_game") or q
        a = obj.get("a") or obj.get("accel") or [0.0, 0.0, 0.0]
        g = obj.get("g") or obj.get("gyro") or [0.0, 0.0, 0.0]
        la = obj.get("la") or obj.get("lacc") or None
        out = {
            "timestamp_us": int(ts),
            "quat": [float(q[i]) if i < len(q) else (1.0 if i == 0 else 0.0) for i in range(4)],
            "quat_game": [float(q6[i]) if i < len(q6) else (1.0 if i == 0 else 0.0) for i in range(4)],
            "quat_accuracy": int(obj.get("qa", 0)),
            "quat_game_accuracy": int(obj.get("q6a", 0)),
            "accel": [float(a[i]) if i < len(a) else 0.0 for i in range(3)],
            "gyro": [float(g[i]) if i < len(g) else 0.0 for i in range(3)],
        }
        if la is not None:
            out["lacc"] = [float(la[i]) if i < len(la) else 0.0 for i in range(3)]
        return out


# =============================================================================
# IMU 采集客户端
# =============================================================================
class IMUClient:
    """
    连接 ESP32+BNO085, 接收并记录 IMU 数据.
    支持 TCP 命令接口 (接受 start_all.py 控制).
    """

    def __init__(self, addr: str, port: int, session: str,
                 position: str = "", output_dir: Path = OUTPUT_DIR,
                 orientation_axis_map: str = None,
                 orientation_baseline_s: float = ORIENTATION_BASELINE_S):
        self.addr = addr
        self.port = port
        self.session = session
        self.position = position
        self.output_dir = Path(output_dir)
        self.orientation_axis_map = orientation_axis_map or ORIENTATION_AXIS_MAPS.get(
            position.lower(), "+x,+y,+z")
        self.orientation_baseline_s = orientation_baseline_s

        self.sock: socket.socket = None
        self.parser = BNO08xParser()
        self.last_recv_time = time.time()  # 上一次收到 socket 数据的 wall 时间

        # 数据
        self.lock = threading.Lock()
        self.running = False
        self.start_wall = None
        self.stop_wall = None
        self.sync_base_ts = None  # 同步基准

        # 时间戳队列 (每个 IMU 样本的 wall-clock)
        self.timestamps: list = []
        self.sensor_time_us: list = []
        self.quat_wxyz: list = []    # list of [w,x,y,z]
        self.quat_game_wxyz: list = []
        self.quat_accuracy: list = []
        self.quat_game_accuracy: list = []
        self.accel_xyz: list = []    # list of [x,y,z]
        self.gyro_xyz: list = []     # list of [x,y,z]
        self.linaccel_xyz: list = []  # 线性加速度 (硬件去重力), list of [x,y,z]
        self.mag_xyz: list = []      # list of [x,y,z] (留空)

        # 事件
        self.events: list = []

        # 可视化队列 (由 viz_imu_live.py 消费)
        self.viz_queue = None

        # TCP 触发 (本地端口, 供 start_all.py 连接)
        self.trigger_port = None
        self.trigger_sock = None
        self.trigger_thread: threading.Thread = None
        self.trigger_active = False

    # ---------------- 连接 ----------------
    def connect(self, timeout: float = 5.0) -> bool:
        """连接到 ESP32 TCP 服务器."""
        try:
            print(color(f"[连接] {self.addr}:{self.port} ...", "yellow"))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(timeout)
            self.sock.connect((self.addr, self.port))
            self.sock.settimeout(1.0)  # 短超时, 便于周期性检查
            print(color(f"[OK] 已连接到 ESP32+BNO085", "green"))
            self.last_recv_time = time.time()
            return True
        except Exception as e:
            print(color(f"[ERR] 连接失败: {e}", "red"))
            self.sock = None
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    # ---------------- 采集控制 ----------------
    def start(self, countdown_s: float = 0, sync_base_ts: float = None):
        """启动采集."""
        with self.lock:
            if self.running:
                return True
            self.running = True
            self.start_wall = time.time()
            self.stop_wall = None
            self.sync_base_ts = sync_base_ts if sync_base_ts is not None else self.start_wall
            self.timestamps = []
            self.sensor_time_us = []
            self.quat_wxyz = []
            self.quat_game_wxyz = []
            self.quat_accuracy = []
            self.quat_game_accuracy = []
            self.accel_xyz = []
            self.gyro_xyz = []
            self.linaccel_xyz = []
            self.mag_xyz = []
            self.events = []
        print(color(f"[OK] IMU 采集已启动, sync_base_ts={self.sync_base_ts:.6f}", "green"))
        return True

    def stop(self):
        with self.lock:
            if not self.running:
                return
            self.running = False
            self.stop_wall = time.time()
        n = len(self.timestamps)
        elapsed = self.stop_wall - self.start_wall if self.start_wall and self.stop_wall else 0
        print(color(f"[OK] IMU 采集已停止: {n} 样本, {elapsed:.2f}s, {len(self.events)} 事件", "green"))

    def mark_event(self, label: str = "", source: str = "cli"):
        with self.lock:
            if not self.running:
                return False
            seq = len(self.events) + 1
            ts = time.time()
            n = len(self.timestamps)
            evt = {
                "seq": seq,
                "wall_ts": ts,
                "sample_idx": n,
                "type": "mark",
                "label": label or f"event_{seq}",
                "source": source,
            }
            self.events.append(evt)
        print(color(f"  >> IMU 事件 #{seq}  sample_idx={n}  ts={ts:.6f}  label={label}", "cyan"))
        return True

    # ---------------- TCP 触发服务器 (供 start_all.py 控制) ----------------
    def start_trigger_server(self, port: int = None):
        """在本地端口监听, 接受 start_all.py 的控制命令."""
        self.trigger_port = port
        self.trigger_active = True
        self.trigger_thread = threading.Thread(target=self._trigger_loop, daemon=True)
        self.trigger_thread.start()
        if port:
            print(color(f"[OK] TCP 触发服务器: 127.0.0.1:{port}", "green"))

    def _trigger_loop(self):
        if self.trigger_port is None:
            return
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", self.trigger_port))
            srv.listen(1)
            srv.settimeout(1.0)
        except Exception as e:
            print(color(f"[WARN] 触发端口 {self.trigger_port} 无法绑定: {e}", "yellow"))
            return

        while self.trigger_active:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            # 单命令交互
            try:
                data = b""
                conn.settimeout(3.0)
                while b"\n" not in data:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                if data.strip():
                    line = data.decode("utf-8", errors="ignore").strip()
                    resp = self._dispatch(line)
                    conn.sendall((json.dumps(resp) + "\n").encode())
                conn.close()
            except Exception:
                try:
                    conn.close()
                except Exception:
                    pass

        try:
            srv.close()
        except Exception:
            pass

    def _dispatch(self, line: str) -> dict:
        try:
            req = json.loads(line)
        except Exception as e:
            return {"ok": False, "err": f"json: {e}"}
        action = req.get("action", "").lower()
        if action == "start":
            ok = self.start(sync_base_ts=req.get("sync_base_ts"))
            return {"ok": ok, "sync_base_ts": self.sync_base_ts,
                    "start_wall": self.start_wall}
        if action == "stop":
            self.stop()
            return {"ok": True, "state": "stopped",
                    "sync_base_ts": self.sync_base_ts,
                    "start_wall": self.start_wall,
                    "n_samples": len(self.timestamps),
                    "n_events": len(self.events)}
        if action == "mark":
            label = req.get("label", "")
            source = req.get("source", "tcp")
            ok = self.mark_event(label=label, source=source)
            return {"ok": ok, "event_seq": len(self.events)}
        if action == "ping":
            return {"ok": True, "state": "running" if self.running else "stopped",
                    "n_samples": len(self.timestamps),
                    "n_events": len(self.events),
                    "sync_base_ts": self.sync_base_ts}
        return {"ok": False, "err": f"unknown: {action}"}

    # ---------------- 接收循环 ----------------
    def receive_loop(self):
        """从 ESP32 接收数据的主循环."""
        buf = b""
        while self.running:
            if not self.sock:
                time.sleep(0.1)
                continue
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    print(color("[WARN] ESP32 连接断开", "yellow"))
                    time.sleep(1.0)
                    # 重连
                    self.close()
                    if self.connect():
                        print(color("[OK] 重新连接到 ESP32", "green"))
                        self.last_recv_time = time.time()
                    continue
                self.last_recv_time = time.time()
                buf += chunk
                # 从 buf 中提取完整帧
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    frame = self._parse_line(line)
                    if frame:
                        self._record_frame(frame)
            except socket.timeout:
                # 超过 5s 没收到任何字节, 可能是 stale socket, 主动重连
                if time.time() - self.last_recv_time > 5.0:
                    print(color("[WARN] 超过 5s 未收到数据,尝试重连 ESP32", "yellow"))
                    self.close()
                    if self.connect():
                        print(color("[OK] 重新连接到 ESP32", "green"))
                        self.last_recv_time = time.time()
                continue
            except Exception:
                if self.running:
                    time.sleep(0.1)
                continue

    def _parse_line(self, line: bytes) -> dict:
        """解析一帧数据, 返回 dict 或 None."""
        if line.startswith(b"{"):
            try:
                obj = json.loads(line.decode("utf-8", errors="ignore"))
                return self._normalize_json(obj)
            except Exception:
                return None
        # 二进制: 32B (无 gyro) 或 44B (有 gyro)
        if len(line) >= 32:
            try:
                ts_us, qw, qx, qy, qz, ax, ay = struct.unpack("<I7f", line[:32])
                result = {
                    "sensor_time_us": ts_us,
                    "quat": [qw, qx, qy, qz],
                    "quat_game": [qw, qx, qy, qz],
                    "accel": [ax, ay, 0.0],
                    "gyro": [0.0, 0.0, 0.0],
                }
                if len(line) >= 44:
                    gx, gy, gz = struct.unpack("<3f", line[32:44])
                    result["gyro"] = [gx, gy, gz]
                return result
            except Exception:
                pass
        return None

    def _normalize_json(self, obj: dict) -> dict:
        ts_us = obj.get("t") or obj.get("time_us") or obj.get("timestamp_us") or 0
        q = obj.get("q") or obj.get("quat") or [1.0, 0.0, 0.0, 0.0]
        q6 = obj.get("q6") or obj.get("quat_game") or q
        a = obj.get("a") or obj.get("accel") or [0.0, 0.0, 0.0]
        g = obj.get("g") or obj.get("gyro") or [0.0, 0.0, 0.0]
        la = obj.get("la") or obj.get("lacc") or None
        out = {
            "sensor_time_us": int(ts_us),
            "quat": [float(q[i]) if i < len(q) else (1.0 if i == 0 else 0.0) for i in range(4)],
            "quat_game": [float(q6[i]) if i < len(q6) else (1.0 if i == 0 else 0.0) for i in range(4)],
            "quat_accuracy": int(obj.get("qa", 0)),
            "quat_game_accuracy": int(obj.get("q6a", 0)),
            "accel": [float(a[i]) if i < len(a) else 0.0 for i in range(3)],
            "gyro": [float(g[i]) if i < len(g) else 0.0 for i in range(3)],
        }
        if la is not None:
            out["lacc"] = [float(la[i]) if i < len(la) else 0.0 for i in range(3)]
        return out

    def _record_frame(self, frame: dict):
        """把一帧写入数据队列,并可选转发给实时可视化."""
        wall_ts = time.time()          # wall-clock 真实时间
        ts_sensor_us = int(frame.get("sensor_time_us", 0))
        quat = frame.get("quat", [1.0, 0.0, 0.0, 0.0])
        quat_game = frame.get("quat_game", quat)
        accel = frame.get("accel", [0.0, 0.0, 0.0])
        gyro = frame.get("gyro", [0.0, 0.0, 0.0])
        lacc = frame.get("lacc")   # 线性加速度 (硬件去重力), 没有则为 None

        with self.lock:
            self.timestamps.append(wall_ts)
            self.sensor_time_us.append(ts_sensor_us)
            self.quat_wxyz.append(quat)
            self.quat_game_wxyz.append(quat_game)
            self.quat_accuracy.append(int(frame.get("quat_accuracy", 0)))
            self.quat_game_accuracy.append(int(frame.get("quat_game_accuracy", 0)))
            self.accel_xyz.append(accel)
            self.gyro_xyz.append(gyro)
            if lacc is not None and self.linaccel_xyz is not None:
                self.linaccel_xyz.append(lacc)
            self.mag_xyz.append([0.0, 0.0, 0.0])  # 当前固件不单独推送原始磁力计
            first_frame = len(self.timestamps) == 1
        if first_frame:
            print(color("[OK] 收到第一帧 IMU 数据", "green"))

        # 实时可视化转发(非阻塞,队列满则丢旧帧)
        if self.viz_queue is not None:
            try:
                # 传朝向(q6) + 加速度 + 陀螺仪 + 线性加速度 (用于实时航位推算画位置轨迹)
                viz_frame = {"wall_ts": wall_ts, "q6": quat_game,
                             "accel": accel, "gyro": gyro}
                if lacc is not None:
                    viz_frame["lacc"] = lacc
                if self.viz_queue.full():
                    try:
                        self.viz_queue.get_nowait()
                    except Exception:
                        pass
                self.viz_queue.put_nowait(viz_frame)
            except Exception:
                pass

    # ---------------- 保存 ----------------
    def save(self):
        out_dir = self.output_dir / self.session
        out_dir.mkdir(parents=True, exist_ok=True)

        with self.lock:
            n = len(self.timestamps)
            ts = np.array(self.timestamps, dtype=np.float64) if n > 0 else np.array([], dtype=np.float64)
            sensor_us = np.array(self.sensor_time_us, dtype=np.uint64) if n > 0 else np.array([], dtype=np.uint64)
            q = np.array(self.quat_wxyz, dtype=np.float32) if n > 0 else np.zeros((0, 4), dtype=np.float32)
            q6 = np.array(self.quat_game_wxyz, dtype=np.float32) if n > 0 else np.zeros((0, 4), dtype=np.float32)
            qa = np.array(self.quat_accuracy, dtype=np.uint8) if n > 0 else np.array([], dtype=np.uint8)
            q6a = np.array(self.quat_game_accuracy, dtype=np.uint8) if n > 0 else np.array([], dtype=np.uint8)
            a = np.array(self.accel_xyz, dtype=np.float32) if n > 0 else np.zeros((0, 3), dtype=np.float32)
            g = np.array(self.gyro_xyz, dtype=np.float32) if n > 0 else np.zeros((0, 3), dtype=np.float32)
            m = np.array(self.mag_xyz, dtype=np.float32) if n > 0 else np.zeros((0, 3), dtype=np.float32)
            la = (np.array(self.linaccel_xyz, dtype=np.float32)
                  if self.linaccel_xyz and len(self.linaccel_xyz) == n
                  else np.zeros((0, 3), dtype=np.float32))

        if n == 0:
            print(color("[WARN] IMU 无数据", "yellow"))
            return None

        prefix = f"imu_{self.position}" if self.position else "imu"
        base = f"{prefix}_{self.session}"

        # 相对同步基准的时间
        elapsed = ts - self.sync_base_ts if self.sync_base_ts else ts - ts[0]

        # ESP32 micros() 是 uint32, 约 71.6 分钟回绕一次. 解包后用于恢复均匀的传感器时间轴.
        sensor_clock_ts = ts.copy()
        if len(sensor_us) == n and np.any(sensor_us):
            raw = sensor_us.astype(np.int64)
            wraps = np.cumsum(np.r_[0, np.diff(raw) < -(1 << 31)], dtype=np.int64)
            unwrapped_us = raw + wraps * (1 << 32)
            sensor_elapsed = (unwrapped_us - unwrapped_us[0]).astype(np.float64) / 1_000_000.0
            sensor_clock_ts = ts[0] + sensor_elapsed
        else:
            sensor_elapsed = elapsed.copy()

        # NPZ
        npz_path = out_dir / f"{base}.npz"
        np.savez_compressed(
            npz_path,
            timestamps=ts,
            timestamps_sensor_clock=sensor_clock_ts,
            sensor_time_us=sensor_us,
            sensor_elapsed_s=sensor_elapsed,
            elapsed_since_sync=elapsed,
            quat_wxyz=q,
            quat_game_wxyz=q6,
            quat_accuracy=qa,
            quat_game_accuracy=q6a,
            accel_xyz=a,
            gyro_xyz=g,
            linaccel_xyz=la,
            mag_xyz=m,
            n_samples=n,
            sample_rate_approx=SAMPLE_RATE_APPROX,
            sync_base_ts=np.float64(self.sync_base_ts or 0.0),
            start_wall=np.float64(self.start_wall or 0.0),
            stop_wall=np.float64(self.stop_wall or 0.0),
            position=self.position,
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
        meta = {
            "session": self.session,
            "position": self.position,
            "device": "BNO085 + ESP32-S3 (Arduino/PlatformIO)",
            "orientation_reports": {
                "quat_wxyz": "9-axis rotation vector (magnetic north + gravity)",
                "quat_game_wxyz": "6-axis game rotation vector (gyro + accel)",
            },
            "sample_rate_hz": SAMPLE_RATE_APPROX,
            "n_samples": n,
            "start_wall": self.start_wall,
            "stop_wall": self.stop_wall,
            "sync_base_ts": self.sync_base_ts,
            "duration_s": (self.stop_wall - self.start_wall) if (self.stop_wall and self.start_wall) else 0,
            "n_events": len(self.events),
            "events": self.events,
        }
        meta_path = out_dir / f"{base}_meta.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print(color(f"[保存] {base}  -> {out_dir}", "green"))
        print(f"       {n} 样本, {len(self.events)} 事件, {meta['duration_s']:.2f}s")

        # 原始数据落盘成功后自动生成姿态轨迹. 转换失败不能影响原始数据保存.
        try:
            trajectory_input = {
                "timestamps": ts,
                "timestamps_sensor_clock": sensor_clock_ts,
                "quat_wxyz": q,
                "quat_game_wxyz": q6,
                "quat_accuracy": qa,
                "quat_game_accuracy": q6a,
                "gyro_xyz": g,
                "sample_rate_approx": np.float32(SAMPLE_RATE_APPROX),
            }
            trajectory, trajectory_meta = build_orientation_trajectory(
                trajectory_input,
                baseline_s=self.orientation_baseline_s,
                axis_map=self.orientation_axis_map,
            )
            orientation_path = out_dir / f"{base}_orientation.npz"
            paths = save_trajectory(
                orientation_path, trajectory, trajectory_meta,
                source_path=npz_path, side=self.position)
            print(color(
                f"[姿态] 自动转换完成  axis-map={self.orientation_axis_map}", "green"))
            for path in paths:
                print(f"       {path.name}")
        except Exception as exc:
            print(color(f"[WARN] 姿态自动转换失败, 原始 NPZ 已保留: {exc}", "yellow"))
        return out_dir


# =============================================================================
# 入口
# =============================================================================
def banner(addr, port, position):
    print(color("=" * 55, "blue"))
    print(color(f"  IMU 采集客户端  ({position})", "bold"))
    print(color(f"  连接: {addr}:{port}", "dim"))
    print(color("  按 Ctrl+C 停止采集并保存", "dim"))
    print(color("=" * 55, "blue"))


def main():
    ap = argparse.ArgumentParser(description="IMU 采集客户端 (BNO085 + ESP32-S3)")
    ap.add_argument("--addr", type=str, default=DEFAULT_ADDR,
                    help=f"ESP32 IP 地址 (默认: {DEFAULT_ADDR})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"TCP 端口 (默认: {DEFAULT_PORT})")
    ap.add_argument("--session", type=str, default=None,
                    help="session 名 (默认: 时间戳)")
    ap.add_argument("--position", type=str, default="left",
                    help="传感器位置 (left/right, 用于文件名前缀)")
    ap.add_argument("--trigger-port", type=int, default=None,
                    help="本地 TCP 触发端口 (供 start_all.py 控制)")
    ap.add_argument("--countdown", type=float, default=0,
                    help="预热倒计时秒数")
    ap.add_argument("--axis-map", default=None,
                    help="姿态轴映射; 默认按 left/right 使用内置标定值")
    ap.add_argument("--baseline-s", type=float, default=ORIENTATION_BASELINE_S,
                    help="自动姿态转换的开头静止归零时长, 默认 2 秒")
    ap.add_argument("--output-dir", type=Path, default=OUTPUT_DIR,
                    help="输出目录 (默认: ./data/output)")
    ap.add_argument("--viz", action="store_true",
                    help="启用实时 3D 姿态可视化窗口 (需要 pyqtgraph + PyQt5)")
    args = ap.parse_args()

    session = args.session or datetime.datetime.now().strftime("imu_%Y%m%d_%H%M%S")

    banner(args.addr, args.port, args.position)

    client = IMUClient(
        addr=args.addr,
        port=args.port,
        session=session,
        position=args.position,
        output_dir=args.output_dir,
        orientation_axis_map=args.axis_map,
        orientation_baseline_s=args.baseline_s,
    )

    # 启动实时可视化(在独立进程中,避免阻塞采集)
    viz_proc = None
    if args.viz:
        # 先在当前解释器检查依赖,避免子进程因环境不一致失败
        try:
            import pyqtgraph.opengl as gl  # noqa: F401
            import PyQt5                  # noqa: F401
        except ImportError as e:
            print(color(f"[WARN] 可视化依赖缺失: {e}", "yellow"))
            print(color("      请运行: pip install pyqtgraph PyQt5 PyOpenGL", "yellow"))
            args.viz = False
        else:
            try:
                from viz_imu_live import start_viz_process
                q, viz_proc = start_viz_process(
                    side=args.position,
                    axis_map=client.orientation_axis_map,
                    baseline_s=args.baseline_s,
                )
                client.viz_queue = q
                print(color(f"[VIZ] 已启动 {args.position} 实时可视化窗口", "cyan"))
            except Exception as e:
                print(color(f"[WARN] 可视化启动失败: {e}", "yellow"))

    # 启动 TCP 触发服务器 (供 start_all.py 控制)
    if args.trigger_port:
        client.start_trigger_server(port=args.trigger_port)

    # 连接到 ESP32
    if not client.connect(timeout=5.0):
        print(color("[ERR] ESP32 连接失败, 退出", "red"))
        return 1

    # 预热倒计时
    if args.countdown > 0:
        for i in range(int(args.countdown), 0, -1):
            print(color(f"  {i} ...", "yellow"))
            time.sleep(1.0)
        print(color("  开始!", "green"))

    if args.trigger_port:
        # 触发模式下等待外部 start 命令
        print(color("[NO-INPUT] 等待 start_all.py 发送 start 命令...", "yellow"))
        try:
            while not client.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(color("\n[EXIT] 未等到 start 命令,退出", "yellow"))
            client.stop()
            client.close()
            if viz_proc:
                viz_proc.terminate()
                viz_proc.join(timeout=1)
            return 0
    else:
        # 独立模式: 立即启动采集
        client.start(countdown_s=args.countdown)

    # 启动接收线程 (必须在 client.start() 之后,否则 running=False 会立即退出)
    recv_thread = threading.Thread(target=client.receive_loop, daemon=True)
    recv_thread.start()

    # 状态打印
    last_n = 0
    last_t = time.time()
    warned_invalid = False
    try:
        while True:
            time.sleep(3.0)
            # 触发模式下外部 stop 命令会设置 running=False,这里退出并执行保存
            if not client.running:
                print(color("[INFO] 收到停止命令,准备保存...", "cyan"))
                break
            with client.lock:
                n = len(client.timestamps)
            elapsed = time.time() - client.start_wall if client.start_wall else 0
            rate = n / elapsed if elapsed > 0.3 else 0
            if n == 0:
                print(color(f"[WARN] 已运行 {elapsed:.1f}s 仍未收到 IMU 数据,请检查 BNO085 接线/接触", "yellow"))
            else:
                # 检查最近 100 帧是否全为默认值(BNO08x 未正常输出)
                invalid = 0
                recent = min(100, n)
                with client.lock:
                    for i in range(n - recent, n):
                        if (client.quat_wxyz[i] == [1.0, 0.0, 0.0, 0.0] and
                                client.accel_xyz[i] == [0.0, 0.0, 0.0] and
                                client.gyro_xyz[i] == [0.0, 0.0, 0.0] and
                                client.quat_accuracy[i] == 0):
                            invalid += 1
                if invalid == recent and recent > 0:
                    print(color(
                        f"[WARN] 已运行 {elapsed:.1f}s,收到 {n} 帧但全为默认值,"
                        "BNO08x 可能未正常工作/接触不良", "yellow"))
                    warned_invalid = True
                else:
                    print(
                        f"{color('[STATUS]', 'dim')}  "
                        f"累计 {n} 样本  "
                        f"已运行 {elapsed:.1f}s  "
                        f"实测速率 {rate:.0f} Hz  "
                        f"事件 {len(client.events)}"
                    )
                    if warned_invalid:
                        print(color("[OK] IMU 数据已恢复正常", "green"))
                        warned_invalid = False
                    last_n = n
                    last_t = time.time()
    except KeyboardInterrupt:
        pass

    # 停止并保存
    client.stop()
    recv_thread.join(timeout=2.0)
    client.close()
    client.save()
    if viz_proc:
        # 先发停止哨兵让可视化窗口立即关闭, 再兜底 terminate
        try:
            client.viz_queue.put_nowait({"stop": True})
        except Exception:
            pass
        viz_proc.join(timeout=2.0)
        if viz_proc.is_alive():
            viz_proc.terminate()
            viz_proc.join(timeout=1)
    print(color("[完成] IMU 采集退出", "green"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
