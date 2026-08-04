"""相机采集器 - 支持 USB 相机和网络相机.

支持:
  - USB 相机 (cv2.VideoCapture)
  - 网络相机 (RTSP/HTTP 流)
  - 真实时间戳记录
  - TCP 命令接口 (接受 orchestrator 控制)

用法:
  python camera_client.py --index 0                    # USB 相机 0
  python camera_client.py --rtsp rtsp://...          # RTSP 流
  python camera_client.py --http http://...           # HTTP 流
  python camera_client.py --trigger-port 8768         # TCP 触发端口
"""

import argparse
import csv
import datetime
import json
import socket
import struct
import sys
import threading
import time
import zipfile
from pathlib import Path
from collections import deque

import numpy as np

# 可选依赖
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    print("[WARN] OpenCV 未安装, 无法使用相机功能")
    print("       安装: pip install opencv-python")


# =============================================================================
# 常量
# =============================================================================
DEFAULT_TRIGGER_PORT = 8768
OUTPUT_DIR = Path(__file__).parent / "data" / "output"

C = {
    "blue": "\033[94m",
    "green": "\033[92m",
    "yellow": "\033[93m",
    "red": "\033[91m",
    "cyan": "\033[96m",
    "bold": "\033[1m",
    "dim": "\033[2m",
    "reset": "\033[0m",
}


def color(s, c):
    return f"{C[c]}{s}{C['reset']}"


# =============================================================================
# 帧缓存 (BGR -> JPEG -> 时间戳)
# =============================================================================
class FrameBuffer:
    """线程安全的帧缓存."""

    def __init__(self, max_size: int = 1000):
        self.max_size = max_size
        self.lock = threading.Lock()
        # (wall_ts, frame_bytes) 列表
        self.frames: deque = deque(maxlen=max_size)

    def push(self, wall_ts: float, frame_bytes: bytes):
        with self.lock:
            self.frames.append((wall_ts, frame_bytes))

    def pop_all(self) -> list:
        """取走所有帧, 返回 [(wall_ts, frame_bytes), ...]."""
        with self.lock:
            frames = list(self.frames)
            self.frames.clear()
            return frames

    def __len__(self) -> int:
        with self.lock:
            return len(self.frames)


# =============================================================================
# 相机采集客户端
# =============================================================================
class CameraClient:
    """
    相机采集器.
    支持 USB 相机或网络流, 持续录制为视频文件或图像序列.
    """

    def __init__(self, source: str, source_type: str = "usb",
                 session: str = None, position: str = "",
                 output_dir: Path = OUTPUT_DIR):
        """
        Args:
            source: 相机源 (设备索引, RTSP URL, HTTP URL)
            source_type: "usb" | "rtsp" | "http"
            session: session 名
            position: 位置标签 (left/right)
            output_dir: 输出目录
        """
        self.source = source
        self.source_type = source_type
        self.session = session or datetime.datetime.now().strftime("cam_%Y%m%d_%H%M%S")
        self.position = position
        self.output_dir = Path(output_dir)

        self.cap = None
        self.frame_buffer = FrameBuffer()

        # 状态
        self.lock = threading.Lock()
        self.running = False
        self.recording = False
        self.start_wall: float = None
        self.stop_wall: float = None
        self.sync_base_ts: float = None

        # 事件
        self.events: list = []

        # 线程
        self.capture_thread: threading.Thread = None
        self.save_thread: threading.Thread = None

        # 视频写入器
        self.writer = None
        self.writer_path: Path = None

        # 帧计数
        self.frame_count = 0
        self.fps_est = 30.0

        # TCP 触发
        self.trigger_port = None
        self.trigger_thread: threading.Thread = None
        self.trigger_active = False

    # ---------------- 连接 ----------------
    def open(self) -> bool:
        """打开相机."""
        if not HAS_CV2:
            print(color("[ERR] OpenCV 未安装", "red"))
            return False

        print(color(f"[打开] {self.source_type}: {self.source}", "yellow"))

        try:
            if self.source_type == "usb":
                self.cap = cv2.VideoCapture(int(self.source))
            else:
                # RTSP 或 HTTP
                self.cap = cv2.VideoCapture(self.source)
                # 设置缓冲减少延迟
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not self.cap.isOpened():
                print(color(f"[ERR] 无法打开相机: {self.source}", "red"))
                return False

            # 获取属性
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            if fps > 0:
                self.fps_est = fps

            print(color(f"[OK] 分辨率: {w}x{h}, FPS: {self.fps_est}", "green"))
            return True

        except Exception as e:
            print(color(f"[ERR] 打开失败: {e}", "red"))
            return False

    def close(self):
        """关闭相机."""
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    # ---------------- 采集控制 ----------------
    def start(self):
        """启动采集."""
        with self.lock:
            if self.running:
                return True
            self.running = True
            self.start_wall = time.time()
            self.stop_wall = None
            self.sync_base_ts = self.start_wall
            self.frame_count = 0
            self.events = []

        print(color(f"[OK] 相机采集已启动, sync_base_ts={self.sync_base_ts:.6f}", "green"))

        # 启动采集线程
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        return True

    def stop(self):
        """停止采集."""
        with self.lock:
            if not self.running:
                return
            self.running = False
            self.stop_wall = time.time()

        # 停止写入器
        if self.writer:
            try:
                self.writer.release()
            except Exception:
                pass
            self.writer = None

        n = self.frame_count
        elapsed = self.stop_wall - self.start_wall if self.start_wall and self.stop_wall else 0
        print(color(f"[OK] 相机采集已停止: {n} 帧, {elapsed:.1f}s, {len(self.events)} 事件", "green"))

    def mark_event(self, label: str = "", source: str = "cli"):
        """标记事件."""
        with self.lock:
            if not self.running:
                return False
            seq = len(self.events) + 1
            ts = time.time()
            evt = {
                "seq": seq,
                "wall_ts": ts,
                "frame_idx": self.frame_count,
                "type": "mark",
                "label": label or f"event_{seq}",
                "source": source,
            }
            self.events.append(evt)
        print(color(f"  >> 相机事件 #{seq}  frame={self.frame_count}  ts={ts:.6f}  label={label}", "cyan"))
        return True

    # ---------------- 采集循环 ----------------
    def _capture_loop(self):
        """持续采集帧, 存入 buffer."""
        fps_time = time.time()
        fps_count = 0

        while self.running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                print(color("[WARN] 相机帧丢失", "yellow"))
                time.sleep(0.01)
                continue

            wall_ts = time.time()
            self.frame_count += 1
            fps_count += 1

            # 估算 FPS
            if fps_count >= 30:
                elapsed = wall_ts - fps_time
                if elapsed > 0:
                    self.fps_est = fps_count / elapsed
                fps_time = wall_ts
                fps_count = 0

            # 编码为 JPEG
            if self.writer is None and self.recording:
                self._open_writer(frame.shape)

            # 写入视频
            if self.writer:
                # 计算相对时间戳 (用于字幕)
                rel_ts = wall_ts - self.sync_base_ts
                # 在帧上添加时间戳 (可选)
                # frame = cv2.putText(frame, f"{rel_ts:.2f}s", (10, 30),
                #                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                self.writer.write(frame)

            # 存入 buffer (保留最近 N 帧)
            if self.recording:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.frame_buffer.push(wall_ts, buf.tobytes())

            time.sleep(0.001)  # 不占用全部 CPU

        # 取走剩余帧
        self._flush_buffer()

    def _open_writer(self, frame_shape):
        """打开视频写入器."""
        out_dir = self.output_dir / self.session
        out_dir.mkdir(parents=True, exist_ok=True)

        prefix = f"cam_{self.position}" if self.position else "cam"
        ext = f"{self.session}.mp4"
        self.writer_path = out_dir / f"{prefix}_{ext}"

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frame_shape[:2]
        self.writer = cv2.VideoWriter(
            str(self.writer_path),
            fourcc,
            self.fps_est,
            (int(w), int(h))
        )
        print(color(f"[写入] {self.writer_path}", "green"))

    def _flush_buffer(self):
        """刷新剩余帧到视频."""
        # buffer 已在 _capture_loop 中写入视频
        pass

    def start_recording(self):
        """开始录制."""
        with self.lock:
            self.recording = True
        print(color("[录制] 开始", "green"))

    def stop_recording(self):
        """停止录制."""
        with self.lock:
            self.recording = False
        print(color("[录制] 结束", "yellow"))

    # ---------------- TCP 触发服务器 ----------------
    def start_trigger_server(self, port: int = None):
        """启动 TCP 触发服务器."""
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
            self.start()
            self.start_recording()
            return {"ok": True, "sync_base_ts": self.sync_base_ts, "start_wall": self.start_wall}
        if action == "stop":
            self.stop_recording()
            self.stop()
            return {"ok": True, "n_frames": self.frame_count}
        if action == "mark":
            label = req.get("label", "")
            source = req.get("source", "tcp")
            ok = self.mark_event(label=label, source=source)
            return {"ok": ok, "event_seq": len(self.events)}
        if action == "ping":
            return {"ok": True, "running": self.running, "recording": self.recording,
                    "n_frames": self.frame_count, "fps": self.fps_est,
                    "sync_base_ts": self.sync_base_ts}
        return {"ok": False, "err": f"unknown: {action}"}

    # ---------------- 保存 ----------------
    def save(self):
        """保存元数据和事件."""
        out_dir = self.output_dir / self.session
        out_dir.mkdir(parents=True, exist_ok=True)

        prefix = f"cam_{self.position}" if self.position else "cam"
        base = f"{prefix}_{self.session}"

        # 元数据
        meta = {
            "session": self.session,
            "position": self.position,
            "source_type": self.source_type,
            "source": self.source,
            "fps": self.fps_est,
            "n_frames": self.frame_count,
            "start_wall": self.start_wall,
            "stop_wall": self.stop_wall,
            "sync_base_ts": self.sync_base_ts,
            "duration_s": (self.stop_wall - self.start_wall) if (self.stop_wall and self.start_wall) else 0,
            "video_path": str(self.writer_path) if self.writer_path else None,
            "n_events": len(self.events),
            "events": self.events,
        }
        meta_path = out_dir / f"{base}_meta.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        # 事件 CSV
        csv_path = out_dir / f"{base}_events.csv"
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["seq", "wall_ts", "frame_idx", "type", "label", "source"])
            for e in self.events:
                w.writerow([e["seq"], f"{e['wall_ts']:.6f}", e.get("frame_idx", 0),
                            e["type"], e["label"], e["source"]])

        print(color(f"[保存] {base} -> {out_dir}", "green"))
        if self.writer_path and self.writer_path.exists():
            print(f"       视频: {self.writer_path.name} ({self.writer_path.stat().st_size / 1024 / 1024:.1f} MB)")
        return out_dir


# =============================================================================
# 入口
# =============================================================================
def banner(source, source_type, position):
    print(color("=" * 55, "blue"))
    print(color(f"  相机采集器 ({position})", "bold"))
    print(color(f"  源: {source_type} -> {source}", "dim"))
    print(color("  按 Ctrl+C 停止采集并保存", "dim"))
    print(color("=" * 55, "blue"))


def main():
    ap = argparse.ArgumentParser(description="相机采集器")
    ap.add_argument("--index", type=int, default=None,
                    help="USB 相机设备索引 (0, 1, ...)")
    ap.add_argument("--rtsp", type=str, default=None,
                    help="RTSP 流地址")
    ap.add_argument("--http", type=str, default=None,
                    help="HTTP 流地址")
    ap.add_argument("--session", type=str, default=None,
                    help="session 名")
    ap.add_argument("--position", type=str, default="",
                    help="位置标签 (left/right)")
    ap.add_argument("--trigger-port", type=int, default=None,
                    help="TCP 触发端口")
    ap.add_argument("--fps", type=float, default=30,
                    help="目标帧率 (默认 30)")
    args = ap.parse_args()

    if not HAS_CV2:
        print(color("[ERR] OpenCV 未安装, 无法运行", "red"))
        return 1

    # 确定源
    if args.index is not None:
        source = str(args.index)
        source_type = "usb"
    elif args.rtsp:
        source = args.rtsp
        source_type = "rtsp"
    elif args.http:
        source = args.http
        source_type = "http"
    else:
        source = "0"  # 默认 USB 相机 0
        source_type = "usb"

    session = args.session or datetime.datetime.now().strftime("cam_%Y%m%d_%H%M%S")
    position = args.position or ""

    banner(source, source_type, position)

    client = CameraClient(
        source=source,
        source_type=source_type,
        session=session,
        position=position,
    )

    # TCP 触发
    if args.trigger_port:
        client.start_trigger_server(port=args.trigger_port)

    # 打开相机
    if not client.open():
        return 1

    # 启动采集
    client.start()
    client.start_recording()

    # 状态打印
    last_frames = 0
    try:
        while True:
            time.sleep(3)
            with client.lock:
                n = client.frame_count
                elapsed = time.time() - client.start_wall if client.start_wall else 0
            rate = n / elapsed if elapsed > 0.3 else 0
            if n != last_frames:
                print(
                    f"{color('[STATUS]', 'dim')}  "
                    f"累计 {n} 帧  "
                    f"已运行 {elapsed:.1f}s  "
                    f"实测 FPS {rate:.1f}  "
                    f"事件 {len(client.events)}"
                )
                last_frames = n
    except KeyboardInterrupt:
        pass

    # 停止并保存
    client.stop_recording()
    client.stop()
    client.save()
    client.close()
    print(color("[完成] 相机采集退出", "green"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
