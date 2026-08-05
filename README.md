# semg

双臂 sEMG + 双腕 IMU 采集系统（客户交付版）

> 配套硬件：`LK-M1299-16CH`（16 通道 sEMG 采集板）+ `FireBeetle 2 ESP32-S3 + GY-BNO08x`（双腕 IMU）

---

## 仓库结构

```
semg/
├── firmware/        # ESP32-S3 固件：BNO085 IMU → WiFi/TCP 推送
├── collector/       # Python 采集上位机（sEMG + IMU + 摄像头）
└── resources/       # 硬件资料、原理图、接线图、参考手册
```

| 目录 | 说明 |
|------|------|
| [`firmware/`](firmware/) | Arduino/PlatformIO 工程。支持左右手、STA/AP 两种 WiFi 模式 |
| [`collector/`](collector/) | 桌面采集工具链。含 sEMG 串口读取、IMU TCP 客户端、实时可视化、自检脚本 |
| [`resources/`](resources/) | LK-M1299 原理图、PCB、侧面图、使用说明书、参考手册、历史软件 |

---

## 快速开始

### 1. 烧录 IMU 固件

```bash
cd firmware
pio run -e imu_left  -t upload   # 左手
pio run -e imu_right -t upload   # 右手
```

默认 STA 模式，WiFi 账号密码写在 [`firmware/src/main.cpp`](firmware/src/main.cpp) 顶部。  
无路由器时可用 AP 模式：`pio run -e imu_left_ap -t upload`。

### 2. 运行采集上位机

```bash
cd collector
pip install pyserial numpy
python collect_cli.py          # 命令行采集
python self_check.py           # 环境自检
```

详细教程见 [`collector/docs/TUTORIAL.md`](collector/docs/TUTORIAL.md)。

---

## 注意

- 这块 ESP32-S3 实际 Flash 为 **4MB**，`platformio.ini` 中已强制 `board_upload.flash_size = 4MB`，不要用 devkitc-1 默认 8MB，否则会启动崩溃。
- Windows 下 `-D` 宏传字符串容易转义失败，WiFi 凭据直接写死在源码顶部，左右手共用。
- `data/` 目录（采集输出）已被 `.gitignore` 排除，不会进仓库。

---

## 关联仓库

- [`ego_vio`](https://github.com/sencangWei/ego_vio) — 本团队的双目/单目 VIO 系统
