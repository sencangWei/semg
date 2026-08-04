"""IMU TCP 原始数据诊断脚本.

连接 ESP32 的 TCP 端口,直接打印收到的原始字节/帧,不解析、不保存.
用来判断是网络问题(收不到包)还是 imu_client 解析问题.
"""

import socket
import sys
import time


def main():
    addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.113.49"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8766
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    print(f"[连接] {addr}:{port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((addr, port))
    sock.settimeout(1.0)
    print("[OK] TCP 已连接,开始接收原始数据...")

    buf = b""
    total_bytes = 0
    total_lines = 0
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                print("[WARN] 对端关闭连接")
                break
            total_bytes += len(chunk)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                total_lines += 1
                print(f"[{total_lines}] {line[:200]}")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        sock.close()

    print(f"\n[统计] {timeout:.0f}s 内收到 {total_bytes} 字节, {total_lines} 行数据")
    if total_lines == 0:
        print("[结论] 没有收到任何数据 -> 网络/固件/硬件问题,imu_client 端没问题")
    else:
        print("[结论] 能收到数据 -> 网络正常,请检查 imu_client.py 解析逻辑")


if __name__ == "__main__":
    main()
