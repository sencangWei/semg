# semg

双臂 sEMG + 双腕 IMU 采集系统（客户交付版）

> 配套硬件：
> - `LK-M1299-16CH`：16 通道 sEMG 采集板（单台 8 通道，两台组合为双臂 16/32 通道）
> - `FireBeetle 2 ESP32-S3 + GY-BNO08x`：双腕 9 轴 IMU，200 Hz 通过 WiFi/TCP 推流
>
> 适用场景：工业抓取 / 康复 / VLA（Vision-Language-Action）训练数据采集。

---

## 1. 仓库结构

```
semg/
├── firmware/        # ESP32-S3 固件：BNO085 IMU → WiFi/TCP 推送
├── collector/       # Python 采集上位机（sEMG + IMU + 摄像头）
└── resources/       # 硬件资料、原理图、PCB、接线图、参考手册
```

| 目录 | 核心内容 |
|------|---------|
| [`firmware/`](firmware/) | Arduino/PlatformIO 工程，支持左右手、STA/AP 两种 WiFi 模式，含电池保护与姿态推流 |
| [`collector/`](collector/) | 桌面采集工具链：`collect_cli.py`（9 阶段标注）、`collect_pure.py`（无标签自由采集）、`self_check.py`（环境自检）、`imu_client.py`、`align.py` |
| [`resources/`](resources/) | LK-M1299 原理图、PCB DXF、侧面图、使用说明书、参考手册、历史单片机/上位机软件 |

---

## 2. 硬件清单与接线

### 2.1 sEMG 采集板 LK-M1299-16CH

| 参数 | 规格 |
|------|------|
| 通道数 | 8 CH / 板，两台组合为 16 CH |
| 分辨率 | 24 bit |
| 采样率 | 250 / 500 / 1000 / 2000 / 4000 / 8000 / 16000 sps |
| 可编程增益 | 1, 2, 4, 6, 8, 12, 24 |
| 输入参考噪声 | 1 µVpp（带宽 70 Hz） |
| 共模抑制比 | -110 dB（典型） |
| 供电 | 单电源 0V/5V 或双电源 ±2.5V，通信电平 1.8V/3.3V/5V |
| 尺寸 | 49.2 × 43 mm |

**佩戴顺序：**

1. 把 16 通道柔性绑带缠到被试者前臂（绑带上有 CH1~CH16 丝印，顺时针绕，起点在手腕上方约 2 cm 处）。
2. 在绑带电极位贴上 Ag/AgCl 一次性电极片（每通道 1 片贴在肌肉隆起最高点，1 片贴附近骨性标志作为参考）。
3. 把绑带 FPC 排线插头插到采集主板。
4. 用 USB 数据线把采集主板接到电脑。

详细接线、脑电/肌电佩戴说明见 [`resources/manual.txt`](resources/manual.txt) 或随板说明书。

### 2.2 IMU 节点（FireBeetle 2 ESP32-S3 + GY-BNO08x）

| 角色 | 默认 IP（STA 静态） | TCP 端口 | 用途 |
|------|------------------|---------|------|
| 左手 | `192.168.113.49` | 8766 | BNO085 姿态 + 加速度 + 陀螺仪 |
| 右手 | `192.168.113.50` | 8767 | 同上 |

**接线：**

- SDA → GPIO1
- SCL → GPIO2
- RST → GPIO4
- 板载 LED → GPIO21

数据格式为 JSON，每帧一行，200 Hz：

```json
{"t":1234567,"q":[w,x,y,z],"q6":[w,x,y,z],"qa":3,"q6a":3,
 "a":[ax,ay,az],"g":[gx,gy,gz],"la":[lx,ly,lz]}
```

其中 `q6`（6 轴 Game Rotation Vector，无磁力计）为推荐默认姿态；`la` 为线性加速度（已去重力）。PC 端 `imu_client.py` 收到后统一打 `time.time()` 墙钟时间戳，后处理 `align.py` 按真实时间对齐。

---

## 3. 快速开始

### 3.1 安装 Python 依赖

```bash
cd collector
pip install pyserial numpy
```

如果下载慢，可用清华镜像：

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyserial numpy
```

### 3.2 环境自检

**第一次使用前务必先跑：**

```bash
python self_check.py
```

正常输出包含 6 个 `[OK]`：

- Python 版本 ≥ 3.8
- `pyserial` / `numpy` 已安装
- `core/` 驱动模块完整
- 串口能枚举到设备
- 与设备握手成功
- 3 秒短采数据健康（CH1/CH9 无复制）

### 3.3 烧录 IMU 固件

```bash
cd firmware
pio run -e imu_left  -t upload   # 左手 STA 模式
pio run -e imu_right -t upload   # 右手 STA 模式
```

无路由器 / 离线调试时，切 AP 模式：

```bash
pio run -e imu_left_ap  -t upload
pio run -e imu_right_ap -t upload
```

> ⚠️ 这块 ESP32-S3 实际 Flash 为 **4MB**。`platformio.ini` 已强制 `board_upload.flash_size = 4MB`，不要用 `esp32-s3-devkitc-1` 默认 8MB，否则会启动崩溃（`spi_flash assert`）。
>
> ⚠️ Windows 下命令行 `-D` 传字符串宏会被引号转义破坏，因此 WiFi 账号密码直接写在 [`firmware/src/main.cpp`](firmware/src/main.cpp) 顶部，左右手共用。

### 3.4 采集 sEMG

#### 方案 A：9 阶段按键标注（`collect_cli.py`）

适合动作阶段固定的实验，例如“抓握木板”。

```bash
python collect_cli.py
```

按数字键 `0-8` 标注：

| 键位 | 阶段 | 中文 |
|------|------|------|
| `0` | rest | 休息 / 静息 |
| `1` | approach | 接近木块 |
| `2` | pregrasp | 准备抓握 |
| `3` | contact | 接触木块 |
| `4` | grasp | 抓握 |
| `5` | lift | 提举 |
| `6` | hold | 保持 |
| `7` | place | 放置 |
| `8` | release | 松开 / 复位 |

一个完整动作流：`0 → 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 0`。

按 `q` 停止并保存。输出到 `collector/data/output/`：

```
emg_<客户>_<场景>_<时间>.npz      # numpy 训练格式
emg_<客户>_<场景>_<时间>.csv      # Excel / MATLAB 通用
emg_<客户>_<场景>_<时间>_annotations.json  # 标注元数据
emg_<客户>_<场景>_<时间>.zip      # 三合一打包，直接回传
```

#### 方案 B：无标签自由采集（`collect_pure.py`，推荐 VLA / 多任务场景）

客户自由执行任意任务，不需要按键盘，由 IMU/相机后处理自动切分动作。

```bash
python collect_pure.py
```

支持：

- 每样本一个 Unix 时间戳
- TCP 外部触发（IMU / 相机同步）
- `m <label>` 手动事件标记
- 双臂 32 通道跨平台运行

输出：

```
emg_<session>.npz              # 合并 EMG + 时间戳
eemg_<session>_events.csv      # 事件/触发标记
emg_<session>_meta.json        # 元数据
emg_<session>.zip              # 打包
```

### 3.5 多设备一键启动

```bash
python start_all.py
```

按 `config.json` 自动拉起 sEMG + 双腕 IMU + 相机，并保存同步数据。

---

## 4. 数据格式

### 4.1 CSV

```csv
# customer=张三,scene=木板抓握,sample_rate=1000,n_channels=16
sample_idx,CH1,CH2,CH3,...,CH16
0,123.4,567.8,...
```

- 单位：µV
- 采样率：1000 Hz（默认）

### 4.2 NPZ

```python
import numpy as np
z = np.load("emg_xxx.npz", allow_pickle=True)
data = z["data"]            # shape (n_samples, 16)
sr   = int(z["sample_rate"])  # 1000
anno = z["annotations"]     # 标注列表（标注版）
```

### 4.3 JSON 元数据

```json
{
  "customer": "张三",
  "scene": "木板抓握",
  "sample_rate": 1000,
  "n_channels": 16,
  "gesture_counts": {"0": 10, "1": 10, ...},
  "annotations": [
    {"gesture_id": 1, "gesture_name": "approach", "start_ts": 1751715012.34, "end_ts": 1751715014.56}
  ]
}
```

---

## 5. 常见问题

| 现象 | 排查 |
|------|------|
| 设备管理器没有 COM 口 | 换一根能传数据的 USB 线；检查 CH340/CP210x 驱动 |
| 串口拒绝访问 | 关闭 Arduino IDE、串口助手等占用程序后重插 |
| 数据全是 0 或一条直线 | 检查 FPC 排线、电极是否过期/皮肤干燥 |
| CH1 与 CH9 相关系数 > 0.99 | 固件 bug 或接线问题，联系供应商 |
| IMU 连不上 | 确认 STA 静态 IP 与路由器同网段；或切 AP 模式 |

---

## 6. 采集质量建议

1. 每个被试者采 5~10 分钟，至少 30 次完整动作。
2. 动作阶段保持 2~3 秒，不要连续按同一个键。
3. 至少 3 名被试者，数据才具有泛化性。
4. 环境远离电焊机、大功率电机等强电磁干扰。
5. 用酒精棉片擦拭电极位，待干后再贴电极。

---

## 7. 文档索引

| 文档 | 内容 |
|------|------|
| [`collector/docs/TUTORIAL.md`](collector/docs/TUTORIAL.md) | `collect_cli.py` 完整教程（9 阶段标注版） |
| [`collector/docs/TUTORIAL_PURE.md`](collector/docs/TUTORIAL_PURE.md) | `collect_pure.py` 完整教程（无标签自由采集版） |
| [`resources/manual.txt`](resources/manual.txt) | LK-M1299 使用说明书 |

---

## 8. 关联仓库

- [`ego_vio`](https://github.com/sencangWei/ego_vio) — 本团队的双目/单目实时 VIO 系统，用于与 sEMG/IMU 数据同步。

---

**仓库版本** v1.0  ·  2026-08
