"""LignoEMG-16CH 多设备一键启动脚本.

用法:
  python start_all.py                     # 读取 config.json, 全自动
  python start_all.py --config my.json   # 指定配置文件
  python start_all.py --dry-run          # 不真正启动, 只打印计划

工作流程:
  1. 读取 config.json
  2. 按 device.enabled 分支启动各设备子进程
  3. 等待所有设备 TCP 就绪 (ping 确认)
  4. 显示 "请准备" 倒计时 (prepare_delay_s 秒)
  5. 同时发 start 命令到所有设备
  6. 进入等待状态 (Ctrl+C 或 q 停止)
  7. 同时发 stop 命令到所有设备
  8. 收集各设备数据到 output_dir

数据输出:
  output_dir/session_YYYYMMDD_HHMMSS/
  ├── emg_<session>.npz
  ├── emg_<session>_events.csv
  ├── emg_<session>_meta.json
  ├── imu_left_<session>.npz
  ├── imu_right_<session>.npz
  ├── sync.json          ← 同步基准 (所有设备共用同一个 sync_base_ts)
  └── session.zip        ← 全部打包

核心设计:
  - 真实时间 (Unix timestamp) 是唯一对齐基准
  - 各设备独立运行, 不依赖时钟同步
  - 倒计时结束后统一发 start, 此时记录 sync_base_ts
  - 事件/动作全部用真实时间戳记录
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import threading
import datetime
import zipfile
import shutil
from pathlib import Path

_HERE = Path(__file__).parent.resolve()
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
# TCP 客户端工具
# =============================================================================
def tcp_send(host: str, port: int, req: dict, timeout: float = 3.0) -> dict:
    """发送一个 JSON 命令到设备, 返回响应 dict."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.sendall((json.dumps(req) + "\n").encode())
        line = b""
        while b"\n" not in line:
            line += sock.recv(4096)
        sock.close()
        return json.loads(line.decode().strip())
    except Exception as e:
        return {"ok": False, "err": str(e)}


def tcp_ping(host: str, port: int, timeout: float = 2.0) -> bool:
    resp = tcp_send(host, port, {"action": "ping"}, timeout=timeout)
    return resp.get("ok", False)


# =============================================================================
# 子进程管理
# =============================================================================
class DeviceProcess:
    """管理一个设备子进程."""

    def __init__(self, name: str, cfg: dict, session: str, output_dir: Path):
        self.name = name
        self.cfg = cfg
        self.session = session
        self.output_dir = output_dir
        self.proc: subprocess.Popen = None
        # 数据平面: IMU 用 ESP32 的 IP/端口, sEMG 无数据平面(TCP 控制即全部)
        self.host = cfg.get("tcp_addr", "127.0.0.1")
        self.port = cfg.get("tcp_port")
        # 控制平面: start_all.py 通过本地 TCP 触发服务控制子进程
        self.control_host = "127.0.0.1"
        self.control_port = cfg.get("trigger_port") or self.port
        self.type = cfg.get("type", "unknown")

    def launch(self, semg_dir: Path, dry_run: bool = False, viz: bool = False):
        if not self.cfg.get("enabled", True):
            print(color(f"[SKIP] {self.name}: 已禁用", "dim"))
            return

        if dry_run:
            print(color(f"[DRY] {self.name}: 应该启动 (type={self.type})", "yellow"))
            return

        if self.type == "lk_m1299":
            # sEMG 采集器
            ports = self.cfg.get("ports", [])
            cmd = [
                sys.executable, str(semg_dir / "collect_pure.py"),
                "--no-input",
                "--tcp-port", str(self.control_port),
                "--session", f"{self.session}",
                "--customer", getattr(sys, "_customer", ""),
                "--scene", getattr(sys, "_scene", ""),
                "--wait-for-start",
                "--output-dir", str(self.output_dir),
            ]
            if ports:
                cmd += ["--ports", ",".join(str(p) for p in ports)]
            self.proc = subprocess.Popen(
                cmd,
                cwd=str(semg_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            print(color(f"[START] {self.name} (PID={self.proc.pid})", "green"))

        elif self.type == "bno085_esp32":
            # IMU 采集器
            cmd = [
                sys.executable, str(_HERE / "imu_client.py"),
                "--addr", self.host,
                "--port", str(self.port),
                "--session", f"{self.session}",
                "--position", self.cfg.get("position", ""),
                "--trigger-port", str(self.control_port),
                "--output-dir", str(self.output_dir.parent),
            ]
            if viz:
                cmd.append("--viz")
            self.proc = subprocess.Popen(
                cmd,
                cwd=str(_HERE),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            print(color(f"[START] {self.name} (PID={self.proc.pid})", "green"))

        elif self.type == "webcam":
            # 相机 (占位: 未来实现)
            print(color(f"[TODO] {self.name}: 相机驱动未实现 (type=webcam)", "yellow"))

        else:
            print(color(f"[SKIP] {self.name}: 未知类型 {self.type}", "yellow"))

    def is_ready(self) -> bool:
        if self.proc is None:
            return True  # dry_run 或禁用
        if self.proc.poll() is not None:
            return False
        if self.control_port is None:
            return True
        return tcp_ping(self.control_host, self.control_port)

    def send_start(self, sync_base_ts: float = None, countdown: float = 0):
        if self.proc is None or self.control_port is None:
            return
        req = {"action": "start", "session": self.session, "countdown": countdown}
        if sync_base_ts is not None:
            req["sync_base_ts"] = sync_base_ts
        resp = tcp_send(self.control_host, self.control_port, req)
        sync_ts = resp.get("sync_base_ts") or resp.get("start_wall")
        return sync_ts

    def send_stop(self):
        if self.proc is None or self.control_port is None:
            return
        return tcp_send(self.control_host, self.control_port, {"action": "stop"})

    def send_mark(self, label: str, source: str = "orchestrator"):
        if self.proc is None or self.control_port is None:
            return
        return tcp_send(self.control_host, self.control_port,
                        {"action": "mark", "label": label, "source": source})

    def wait_until_ready(self, timeout: float = 15.0, poll_interval: float = 0.5):
        """等待设备就绪 (TCP ping 成功)."""
        if self.proc is None:
            return True
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                print(color(f"[WARN] {self.name} 进程已退出", "yellow"))
                return False
            if self.is_ready():
                return True
            time.sleep(poll_interval)
        print(color(f"[WARN] {self.name} 等待就绪超时", "yellow"))
        return False

    def terminate(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            except Exception:
                pass

    def is_alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


# =============================================================================
# 主流程
# =============================================================================
def load_config(config_path: Path) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def print_banner(cfg: dict, session: str):
    print(color("=" * 60, "blue"))
    print(color("  LignoEMG-16CH  多设备一键启动", "bold"))
    print(color(f"  Session: {session}", "dim"))
    print(color("=" * 60, "blue"))

    print("\n  启用设备:")
    for name, dev in cfg.get("devices", {}).items():
        if dev.get("enabled"):
            desc = dev.get("description", "")
            print(f"    [{name}] {desc}")
    print()


def countdown_display(seconds: float):
    """显示倒计时, 返回 True 表示正常倒计时结束, False 表示被中断."""
    print(color(f"\n  准备时间: {seconds:.0f} 秒", "yellow"))
    print(color("  客户请准备...", "yellow"))
    for i in range(int(seconds), 0, -1):
        print(color(f"    {i} ...", "yellow"))
        time.sleep(1.0)
    print(color("    开始!", "green"))
    return True


def collect_sync_info(devices: list, stop_responses: dict = None) -> dict:
    """收集各设备同步信息. 优先使用 stop 命令响应, 否则回退 ping."""
    stop_responses = stop_responses or {}
    info = {}
    for dev in devices:
        # 禁用的设备直接标记
        if not dev.cfg.get("enabled", True):
            info[dev.name] = {
                "sync_base_ts": None,
                "start_wall": None,
                "state": "disabled",
                "n_samples": 0,
                "n_events": 0,
            }
            continue

        # 优先用 stop 响应(子进程 stop 后可能已关闭 TCP,ping 会失败)
        resp = stop_responses.get(dev.name)
        if resp and resp.get("ok"):
            info[dev.name] = {
                "sync_base_ts": resp.get("sync_base_ts"),
                "start_wall": resp.get("start_wall"),
                "state": resp.get("state", "stopped"),
                "n_samples": resp.get("n_samples", 0),
                "n_events": resp.get("n_events", 0),
            }
            continue

        # 回退: 尝试 ping
        if dev.control_port:
            resp = tcp_send(dev.control_host, dev.control_port, {"action": "ping"}, timeout=2.0)
            info[dev.name] = {
                "sync_base_ts": resp.get("sync_base_ts"),
                "start_wall": resp.get("start_wall"),
                "state": resp.get("state", "unknown"),
                "n_samples": resp.get("n_samples", 0),
                "n_events": resp.get("n_events", 0),
            }
    return info


def keyloop_stdin():
    """跨平台读键盘 (Windows/Linux/macOS)."""
    if sys.platform == "win32":
        import msvcrt
        while True:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    k = ch.decode("utf-8", errors="ignore")
                except Exception:
                    k = ""
                yield k
            time.sleep(0.05)
    else:
        import select
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                yield line


def run(cfg: dict, args):
    # ---------- session ----------
    session = args.session or cfg.get("session") or \
               datetime.datetime.now().strftime("session_%Y%m%d_%H%M%S")
    output_dir = Path(cfg.get("output_dir", _HERE / "data" / "output"))
    session_dir = output_dir / session
    prepare_s = cfg.get("prepare_delay_s", 5)

    print_banner(cfg, session)

    # ---------- 实例化设备 ----------
    devices = []
    for name, dev_cfg in cfg.get("devices", {}).items():
        dev = DeviceProcess(name, dev_cfg, session, session_dir)
        devices.append(dev)

    # 确保 session 目录存在
    session_dir.mkdir(parents=True, exist_ok=True)

    # ---------- 启动所有设备 ----------
    print(color("--- 启动设备 ---", "bold"))
    semg_dir = _HERE
    for dev in devices:
        dev.launch(semg_dir, dry_run=args.dry_run, viz=args.viz)

    if args.dry_run:
        print(color("\n[DRY RUN] 以上是要启动的设备计划", "yellow"))
        return 0

    # ---------- 等待所有设备就绪 ----------
    print(color("\n--- 等待设备就绪 ---", "bold"))
    all_ready = True
    ready_results = [None] * len(devices)

    def wait_one(idx: int, dev: DeviceProcess):
        if not dev.cfg.get("enabled", True):
            ready_results[idx] = True
            return
        ok = dev.wait_until_ready(timeout=20.0)
        if ok:
            print(color(f"  [OK] {dev.name} 就绪", "green"))
        else:
            print(color(f"  [FAIL] {dev.name} 未就绪", "red"))
        ready_results[idx] = ok

    threads = [threading.Thread(target=wait_one, args=(i, dev), daemon=True)
               for i, dev in enumerate(devices)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    for ok in ready_results:
        if not ok:
            all_ready = False
            break

    if not all_ready:
        print(color("\n[ERR] 有设备未就绪, 是否强制继续? (y/N)", "red"))
        try:
            ch = input("> ").strip().lower()
            if ch != "y":
                for dev in devices:
                    dev.terminate()
                return 1
        except EOFError:
            for dev in devices:
                dev.terminate()
            return 1

    # ---------- 预热倒计时 ----------
    print(color(f"\n--- 预热倒计时 ({prepare_s}s) ---", "bold"))
    print(color("  设备已就绪,倒计时结束后统一开始采集...", "dim"))
    countdown_display(prepare_s)

    # ---------- 同时发 start ----------
    print(color("\n--- 发送 start 命令 ---", "bold"))
    global_sync_base = time.time()
    sync_timestamps = []
    for dev in devices:
        if dev.cfg.get("enabled") and dev.control_port:
            ts = dev.send_start(sync_base_ts=global_sync_base, countdown=0)
            if ts:
                sync_timestamps.append(ts)
                print(color(f"  [->] {dev.name} start, sync_base_ts={ts:.6f}", "green"))
            else:
                print(color(f"  [??] {dev.name} start (无 sync 响应)", "yellow"))
        else:
            print(color(f"  [--] {dev.name} 跳过", "dim"))

    print(color(f"\n  全局 sync_base_ts = {global_sync_base:.6f}", "cyan"))
    print(color("  (所有设备以此真实时间为基准对齐)\n", "dim"))

    # ---------- 等待用户结束 ----------
    print(color("--- 采集中 ---", "bold"))
    print(color("  Ctrl+C 或 输入 q 停止采集", "dim"))
    print(color("  输入 m <label> 标记事件 (例: m pick_apple)\n", "dim"))

    stop_requested = threading.Event()
    key_active = threading.Event()
    key_active.set()

    def status_printer():
        while not stop_requested.is_set():
            elapsed = time.time() - global_sync_base
            parts = [f"已采集 {elapsed:.1f}s"]
            for dev in devices:
                if not dev.cfg.get("enabled") or not dev.control_port:
                    continue
                resp = tcp_send(dev.control_host, dev.control_port,
                                {"action": "ping"}, timeout=1.0)
                if resp.get("ok"):
                    parts.append(f"{dev.name}:{resp.get('n_samples', 0)}")
                else:
                    parts.append(f"{dev.name}:?")
            print(color("  |  ".join(parts), "dim"))
            stop_requested.wait(2.0)

    status_thread = threading.Thread(target=status_printer, daemon=True)
    status_thread.start()

    def keyboard_watcher():
        key_active.wait()
        if sys.platform == "win32":
            import msvcrt
            while key_active.is_set():
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    try:
                        k = ch.decode("utf-8", errors="ignore")
                    except Exception:
                        k = ""
                    if k.lower() == "q" or k == "\x03":
                        stop_requested.set()
                        return
                    if k == "m":
                        # 读后续的 label
                        label_line = ""
                        while key_active.is_set():
                            if msvcrt.kbhit():
                                lc = msvcrt.getch()
                                try:
                                    lc_k = lc.decode("utf-8", errors="ignore")
                                except Exception:
                                    lc_k = ""
                                if lc_k == "\r":
                                    label = label_line.strip()
                                    break
                                label_line += lc_k
                            time.sleep(0.01)
                        else:
                            break
                        for dev in devices:
                            if dev.cfg.get("enabled") and dev.control_port:
                                dev.send_mark(label, source="orchestrator")
                        print(color(f"  >> 标记: {label}", "cyan"))
                time.sleep(0.01)
        else:
            import select
            while key_active.is_set():
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    try:
                        line = sys.stdin.readline()
                    except EOFError:
                        break
                    if not line:
                        break
                    line = line.strip()
                    if line.lower() == "q":
                        stop_requested.set()
                        return
                    if line.lower().startswith("m "):
                        label = line[2:].strip()
                        for dev in devices:
                            if dev.cfg.get("enabled") and dev.control_port:
                                dev.send_mark(label, source="orchestrator")
                        print(color(f"  >> 标记: {label}", "cyan"))

    kb_thread = threading.Thread(target=keyboard_watcher, daemon=True)
    kb_thread.start()

    try:
        stop_requested.wait()
    except KeyboardInterrupt:
        stop_requested.set()
    finally:
        key_active.clear()
        kb_thread.join(timeout=1.0)
        status_thread.join(timeout=1.0)

    # ---------- 停止所有设备 ----------
    print(color("\n--- 发送 stop 命令 ---", "bold"))
    stop_responses = {}
    for dev in devices:
        if dev.cfg.get("enabled") and dev.control_port:
            resp = dev.send_stop()
            stop_responses[dev.name] = resp
            print(color(f"  [->] {dev.name} stop", "yellow"))
        else:
            print(color(f"  [--] {dev.name} 跳过", "dim"))

    # ---------- 保存 sync.json (在子进程退出前用 stop 响应收集状态) ----------
    print(color("\n--- 保存同步信息 ---", "bold"))
    sync_info = {
        "session": session,
        "global_sync_base_ts": global_sync_base,
        "global_sync_base_iso": datetime.datetime.fromtimestamp(global_sync_base).isoformat(),
        "prepare_delay_s": prepare_s,
        "devices": collect_sync_info(devices, stop_responses=stop_responses),
    }
    sync_path = session_dir / "sync.json"
    with open(sync_path, "w", encoding="utf-8") as f:
        json.dump(sync_info, f, ensure_ascii=False, indent=2)
    print(color(f"  sync.json -> {sync_path}", "green"))

    # 等待子进程退出
    print(color("\n--- 等待设备退出 ---", "bold"))
    time.sleep(1.5)
    for dev in devices:
        dev.terminate()
        if dev.is_alive():
            dev.proc.kill()
        if dev.proc and dev.proc.stdout:
            # 打印子进程输出,方便排查可视化等问题
            try:
                lines = dev.proc.stdout.readlines()
                if lines:
                    print(color(f"  {dev.name} stdout:", "dim"))
                    for line in lines[:5]:
                        print(color(f"    {line.rstrip()}", "dim"))
                    if len(lines) > 30:
                        print(color(f"    ... ({len(lines) - 30} 行省略) ...", "dim"))
                    for line in lines[-25:]:
                        print(color(f"    {line.rstrip()}", "dim"))
            except Exception:
                pass
        print(color(f"  [OK] {dev.name} 已停止", "green"))

    # ---------- 打包 ----------
    zip_path = session_dir / f"{session}.zip"
    print(color(f"\n--- 打包 -> {zip_path} ---", "bold"))
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in session_dir.iterdir():
            if f.name.endswith(".zip"):
                continue
            zf.write(f, arcname=f.name)
    print(color(f"  打包完成: {zip_path} ({zip_path.stat().st_size / 1024:.1f} KB)", "green"))

    print(color("\n[完成] 多设备采集结束", "green"))
    print(color(f"  数据目录: {session_dir}", "dim"))
    print(color(f"  打包文件: {zip_path}", "dim"))
    return 0


# =============================================================================
# 入口
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description="LignoEMG-16CH 多设备一键启动")
    ap.add_argument("--config", type=str, default=None,
                    help="配置文件路径 (默认: config.json)")
    ap.add_argument("--dry-run", action="store_true",
                    help="不真正启动, 只打印计划")
    ap.add_argument("--session", type=str, default=None,
                    help="session 名 (默认读取 config 或时间戳)")
    ap.add_argument("--viz", dest="viz", action="store_true", default=True,
                    help="为每个启用的 IMU 打开实时 3D 姿态可视化窗口 (默认开启,需要 pyqtgraph + PyQt5)")
    ap.add_argument("--no-viz", dest="viz", action="store_false",
                    help="关闭 IMU 实时 3D 可视化窗口")
    args = ap.parse_args()

    # 找 config
    if args.config:
        cfg_path = Path(args.config)
    else:
        cfg_path = _HERE / "config.json"

    if not cfg_path.exists():
        print(color(f"[ERR] 配置文件不存在: {cfg_path}", "red"))
        print(color("    请先复制 config.example.json 并修改", "red"))
        return 1

    cfg = load_config(cfg_path)
    if args.session:
        cfg["session"] = args.session

    sys._customer = cfg.get("customer", "")
    sys._scene = cfg.get("scene", "")

    try:
        return run(cfg, args)
    except KeyboardInterrupt:
        print(color("\n已退出", "yellow"))
        return 0


if __name__ == "__main__":
    sys.exit(main())
