"""LignoEMG-16CH 客户环境自检脚本 (含 collect_pure 适配).

按顺序检测:
1. Python 版本
2. 关键依赖 (pyserial / numpy)
3. core/ 下的驱动模块
4. 串口枚举
5. 与设备的握手 (发送采样参数, 读取状态)
6. 通道数据健康度 (3 秒短采, 检查 CH1/CH9 是否重复)
7. collect_pure.py 脚本可用性 + 关键 API 导入测试

退出码: 0 = 全部通过, 非 0 = 有项目失败.
"""
import os
import sys
import time
import struct
import traceback
from pathlib import Path

_HERE = Path(__file__).parent.resolve()
sys.path.insert(0, str(_HERE / "core"))

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"


def ok(msg):   print(f"  {GREEN}[OK]{RESET}    {msg}")
def warn(msg): print(f"  {YELLOW}[WARN]{RESET}  {msg}")
def err(msg):  print(f"  {RED}[FAIL]{RESET}  {msg}")
def head(msg): print(f"\n{BLUE}{BOLD}== {msg} =={RESET}")


def check_python():
    head("1) Python 环境")
    v = sys.version_info
    print(f"  版本: Python {v.major}.{v.minor}.{v.micro}")
    if (v.major, v.minor) >= (3, 8):
        ok("Python 版本 >= 3.8")
        return True
    err("需要 Python 3.8 或以上")
    return False


def check_deps():
    head("2) 第三方依赖")
    all_ok = True
    for mod_name, pkg_name in [("serial", "pyserial"), ("numpy", "numpy")]:
        try:
            __import__(mod_name)
            ok(f"{pkg_name} 已安装")
        except ImportError:
            err(f"缺少 {pkg_name}, 请运行:  pip install {pkg_name}")
            all_ok = False
    return all_ok


def check_core():
    head("3) 驱动模块 (core/)")
    expected = ["ads129x_cmd.py", "ads129x_data.py",
                "emg_filter.py", "eeg_filter.py", "ecg_filter.py"]
    all_ok = True
    for f in expected:
        p = _HERE / "core" / f
        if p.exists():
            ok(f"core/{f}")
        else:
            err(f"core/{f} 缺失")
            all_ok = False
    if all_ok:
        try:
            from ads129x_cmd import ADS129X_Cmd
            from ads129x_data import ADS129X_Data
            ok("驱动模块导入测试通过")
        except Exception as e:
            err(f"驱动模块导入失败: {e}")
            all_ok = False
    return all_ok


def check_serial_ports(skip_bluetooth=True):
    head("4) 串口枚举")
    try:
        from serial.tools import list_ports
    except ImportError:
        err("pyserial 未安装, 跳过")
        return False, None
    ports = sorted(list_ports.comports())
    if skip_bluetooth:
        bt = [p for p in ports if "BTHENUM" in (p.hwid or "")]
        if bt:
            print(f"  (已跳过 {len(bt)} 个蓝牙串口: {', '.join(p.device for p in bt)})")
        ports = [p for p in ports if "BTHENUM" not in (p.hwid or "")]
    if sys.platform != "win32":
        ports = [p for p in ports if not p.device.startswith("/dev/ttyS")]
    if not ports:
        warn("未发现任何串口, 请检查 USB 是否插入")
        return False, None
    for p in ports:
        print(f"  - {p.device}  ({p.description})  hwid={p.hwid or '?'}")
    print(f"  共 {len(ports)} 个串口")
    if len(ports) == 2:
        ok("双臂模式就绪, 将依次测试两台设备")
    return True, ports


def try_handshake(port: str, timeout: float = 3.0) -> bool:
    import serial
    from ads129x_cmd import ADS129X_Cmd
    from ads129x_data import ADS129X_Data
    print(f"  正在与 {port} 握手 ...")
    try:
        ser = serial.Serial(
            port=port, baudrate=2_000_000, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1,
        )
        if hasattr(ser, "set_buffer_size"):
            ser.set_buffer_size(rx_size=102400, tx_size=65536)
        try:
            ser.setRTS(False)
            ser.setDTR(False)
        except (AttributeError, OSError):
            pass
    except Exception as e:
        err(f"打开串口失败: {e}")
        return False

    ads = ADS129X_Data()
    try:
        ser.write(bytes(ADS129X_Cmd.set_sample_par_cmd("1000sps", "±375mV", 0x00FF)))
        time.sleep(0.3)
        ser.write(bytes(ADS129X_Cmd.start_collect_cmd()))
        deadline = time.time() + timeout
        n = 0
        while time.time() < deadline:
            avail = ser.in_waiting
            if avail > 0:
                data = ser.read(min(avail, 4096))
                if data:
                    ads.parse_data(data)
            else:
                time.sleep(0.01)
            with_n = len(ads.emg_raw_data[0]) if ads.emg_raw_data else 0
            if with_n > n:
                n = with_n
                if n >= 200:
                    break

        ser.write(bytes(ADS129X_Cmd.stop_collect_cmd()))
        time.sleep(0.1)
        sr = ads.sample_rate
        chs = len([1 for c in ads.emg_raw_data if len(c) > 0])
        print(f"  收到样本数: {n}, 反馈采样率: {sr} Hz, 通道数: {chs}")
        if n < 50:
            err("数据量过少, 设备可能未连接或 baudrate 不匹配")
            return False
        ok(f"握手成功, 设备已就绪 ({sr} Hz)")
        if chs >= 16 and len(ads.emg_raw_data[0]) >= 1000 and len(ads.emg_raw_data[8]) >= 1000:
            try:
                import numpy as np
                a = np.array(ads.emg_raw_data[0][:1000])
                b = np.array(ads.emg_raw_data[8][:1000])
                if len(a) == 1000 and len(b) == 1000 and a.std() > 0 and b.std() > 0:
                    corr = float(np.corrcoef(a, b)[0, 1])
                    if corr > 0.99:
                        warn(f"CH1 与 CH9 相关系数 {corr:.3f} > 0.99, 可能存在通道复制/接线问题")
                    else:
                        ok(f"CH1 vs CH9 相关系数 {corr:.3f} (健康)")
            except Exception as e:
                warn(f"通道重复检查失败: {e}")
        return True
    except Exception as e:
        traceback.print_exc()
        err(f"握手过程出错: {e}")
        return False
    finally:
        ser.close()


def check_collect_pure():
    """检测 collect_pure.py 脚本可用性."""
    head("7) 纯采集脚本 (collect_pure.py)")
    p = _HERE / "collect_pure.py"
    if not p.exists():
        err("collect_pure.py 不存在")
        return False
    ok("collect_pure.py 文件存在")

    # 语法
    try:
        import ast
        ast.parse(p.read_text(encoding="utf-8"))
        ok("collect_pure.py 语法检查通过")
    except SyntaxError as e:
        err(f"语法错误: {e}")
        return False

    # 导入测试 (不真正打开串口)
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("collect_pure", p)
        if spec is None or spec.loader is None:
            err("无法加载模块 spec")
            return False
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        # 关键类 / 函数
        for name in ("PureCollector", "OUTPUT_DIR", "N_CHANNELS_PER_DEVICE", "SAMPLE_RATE_HZ"):
            if not hasattr(mod, name):
                err(f"缺少关键符号: {name}")
                return False
        ok("PureCollector / OUTPUT_DIR / 常量 导入成功")

        # 关键方法
        collector_cls = mod.PureCollector
        for method in ("start", "stop", "save", "mark_event",
                       "_start_tcp_server", "_tcp_dispatch"):
            if not hasattr(collector_cls, method):
                err(f"PureCollector 缺少方法: {method}")
                return False
        ok("PureCollector 关键方法齐全 (start/stop/save/mark/tcp)")

        return True
    except Exception as e:
        traceback.print_exc()
        err(f"导入失败: {e}")
        return False


def main():
    import argparse
    ap = argparse.ArgumentParser(description="LignoEMG-16CH 环境自检")
    ap.add_argument("--port", type=str, default=None,
                    help="只测试指定串口(如 --port COM7), 跳过枚举和其他口")
    args = ap.parse_args()

    print(f"{BLUE}{BOLD}")
    print("=" * 60)
    print("  LignoEMG-16CH  环境自检工具  v1.1")
    print("=" * 60)
    print(f"{RESET}")

    results = []
    results.append(check_python())
    results.append(check_deps())
    results.append(check_core())

    if args.port:
        head(f"4) 串口测试 (指定端口: {args.port})")
        results.append(try_handshake(args.port, timeout=3.0))
    else:
        port_ok, ports = check_serial_ports(skip_bluetooth=True)
        if port_ok and ports:
            if len(ports) == 1:
                results.append(try_handshake(ports[0].device, timeout=3.0))
            else:
                print(f"\n  检测到 {len(ports)} 个串口, 将依次测试:")
                for i, p in enumerate(ports, 1):
                    print(f"    {i}. {p.device}")
                for p in ports:
                    ok_flag = try_handshake(p.device, timeout=3.0)
                    results.append(ok_flag)
        else:
            results.append(False)

    # 第 7 项: 纯采集脚本
    results.append(check_collect_pure())

    print()
    print(f"{BLUE}{BOLD}== 总结 =={RESET}")
    passed = sum(1 for r in results if r)
    total = len(results)
    if passed == total:
        print(f"  {GREEN}{BOLD}全部通过 ({passed}/{total}){RESET} - 可以开始采集")
        print()
        print("  推荐命令:")
        print("    python collect_pure.py                     # 交互模式")
        print("    python collect_pure.py --tcp-port 8765     # TCP 触发模式")
        return 0
    else:
        print(f"  {YELLOW}{BOLD}{passed}/{total} 通过{RESET} - 请按上方 [FAIL] 提示修复后重试")
        return 1


if __name__ == "__main__":
    sys.exit(main())
