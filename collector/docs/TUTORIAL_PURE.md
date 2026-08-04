# LignoEMG-16CH 纯 sEMG 采集器使用教程 (v3.0 无标签版)

> 面向 **多种分拣 / 抓取 / 放置 / VLA 训练** 场景
> 配套硬件: LK-M1299-16CH × 1-2 台
> 脚本版本: collect_pure.py v3.0

---

## 与旧版 (collect_cli.py) 的区别

| | collect_cli.py (旧) | collect_pure.py (新) |
|---|---|---|
| 动作标签 | 强制 0-8 按键标注 | **不做任何标注** |
| 客户操作 | 必须按 0-8 标记阶段 | **自由执行任意任务** |
| 时间戳 | 只有 start_ts, 1 个值 | **每个样本一个时间戳** |
| 外部触发 | 不支持 | **TCP 触发** (IMU/相机) |
| 事件标记 | 无 | **mark 命令, 可选 label** |
| 跨平台 | 仅 Windows (`msvcrt`) | **跨平台** (`input` + TCP) |

**为什么去掉标签**: 你交付给客户做多种分拣任务 (抓苹果、捡瓶子、分大小), 动作序列不固定,
动作分类应由 **IMU/相机后处理** 自动识别, 不是让客户按键盘。

---

## 1. 文件清单

```
customer_collector/
├── collect_pure.py            ← sEMG 纯采集器 (无标签)
├── collect_cli.py             ← 旧版带标签采集器 (保留兼容)
├── start_all.py              ← 多设备一键启动脚本 (sEMG + IMU + 相机)
├── imu_client.py             ← IMU 采集器客户端 (PC 端, 连接 ESP32+BNO085)
├── orientation_trajectory.py ← 四元数转腕部载体姿态轨迹
├── align.py                  ← 后处理对齐工具 (按真实时间戳同步切分)
├── config.json               ← 设备配置文件
├── self_check.py            ← 环境自检
├── core/                     ← 串口 / 协议 / 滤波模块
│   ├── ads129x_cmd.py
│   ├── ads129x_data.py
│   ├── emg_filter.py
│   ├── eeg_filter.py
│   ├── ecg_filter.py
│   ├── iir_filter.py
│   └── median_filter.py
├── data/
│   └── output/               ← 采集数据保存位置
└── docs/
    └── TUTORIAL_PURE.md     ← 本文档
```

---

## 2. 快速开始 (3 步)

### Step 1: 自检环境

```bash
cd customer_collector
python self_check.py
```

预期看到 6 个 `[OK]`, 包括 Python 版本、pyserial / numpy、驱动模块、串口、握手成功。

### Step 2: 启动采集器

```bash
python collect_pure.py
```

按提示:
1. 输入客户姓名 / 编号 (例: `S001`)
2. 输入采集场景说明 (例: `task1_pick_apple_round1`)
3. 选择端口 (默认自动)

采集启动后, 终端会显示:
```
[OK] 采集已启动 (双臂 32ch @ 1000Hz)
      客户自由操作, 不需要按任何键
      终端命令: m <label> 标记事件 / q 停止
```

**客户就可以开始自由执行任务了**, 不需要按任何键。

### Step 3: 停止并保存

采集完成后, 在终端输入:
```
> q
```

数据自动保存到 `data/output/`:
```
emg_<session>.npz      ← 合并 EMG (numpy), 含时间戳
emg_<session>_events.csv   ← 事件标记
emg_<session>_meta.json    ← 元数据
emg_<session>.zip      ← 打包 (回传这一个)
```

---

## 3. 数据格式

### 3.1 NPZ 主文件

```python
import numpy as np
d = np.load("emg_S001_xxx.npz", allow_pickle=True)

data         # (T, N) float32, 左臂 CH1-16 + 右臂 CH1-16
timestamps   # (T,)   float64, Unix 秒 (秒级精度), 单调递增
sample_idx   # (T,)   int32,  从 0 开始
sample_rate  # 标量 1000
n_channels   # 16 或 32
n_devices    # 1 或 2
arm_labels   # ["L"] 或 ["L", "R"]
start_wall   # 采集开始 wall-clock
stop_wall    # 采集停止 wall-clock
events       # 事件数组 (mark / 外部触发)
```

### 3.2 切分任意时刻的 EMG

```python
# 例: 切出从 t=12.345s 到 t=15.678s 的数据
ts = d["timestamps"]
mask = (ts >= 12.345) & (ts < 15.678)
segment = d["data"][mask]   # (n, 32)
sample_range = d["sample_idx"][mask]
print(f"切出 {segment.shape[0]} 样本, 起始 sample_idx={sample_range[0]}")
```

### 3.3 事件 CSV

```csv
seq,wall_ts,sample_idx,type,label,source
1,1720345678.123456,12000,mark,pick_apple,cli
2,1720345679.456789,13456,mark,place_apple,tcp
```

- `seq`: 事件序号 (从 1 开始)
- `wall_ts`: Unix 秒, 微秒精度
- `sample_idx`: 触发时的 EMG 样本序号
- `label`: 自定义标签 (例: 物体名 / 任务名)
- `source`: `cli` (键盘) 或 `tcp` (外部)

---

## 4. 终端命令

| 命令 | 说明 |
|------|------|
| `m <label>` | 标记一个事件, `<label>` 可选 (例: `m pick_apple`) |
| `q`         | 停止采集并保存 |
| `Ctrl+C`    | 强制停止 (会自动保存) |

**事件有什么用**: 给某一时刻打一个时间锚点。
例如客户每次拿起苹果的瞬间按一下 `m pick_apple`,
将来后处理可以用 `events.csv` 的 `sample_idx` 列直接定位 EMG 区段。

---

## 5. TCP 外部触发 (IMU / 相机联动)

### 5.1 启动

```bash
python collect_pure.py --tcp-port 8765 --no-input
```

- `--tcp-port 8765`: 启动 TCP 服务器, 监听 127.0.0.1:8765
- `--no-input`: 不读键盘, 纯靠 TCP 命令驱动

### 5.2 协议

客户端 (相机 / IMU) 用 TCP 连上 `127.0.0.1:8765`, 发 JSON 行:

```json
{"action": "start", "session": "task_2026_07_07_01"}
{"action": "mark",   "label": "pick_apple"}
{"action": "mark",   "label": "place"}
{"action": "ping"}
{"action": "stop"}
```

每行一个 JSON, **以换行 `\n` 结尾**。服务器回 JSON 行响应。

### 5.3 响应示例

```
→ {"action": "start"}
← {"ok": true, "state": "collecting"}

→ {"action": "mark", "label": "pick_apple"}
← {"ok": true, "event_seq": 1}

→ {"action": "ping"}
← {"ok": true, "state": "collecting", "n_samples": 12500, "n_events": 1}

→ {"action": "stop"}
← {"ok": true, "state": "stopped"}
```

### 5.4 Python 客户端示例

```python
import socket, json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 8765))

def send(req):
    sock.sendall((json.dumps(req) + "\n").encode())
    line = sock.recv(4096).decode().strip()
    return json.loads(line)

print(send({"action": "start", "session": "task_001"}))
# 客户开始做任务...
print(send({"action": "mark", "label": "pick_apple"}))
print(send({"action": "mark", "label": "place"}))
print(send({"action": "stop"}))
sock.close()
```

### 5.5 与 IMU/相机对齐

IMU/相机通过 TCP 在动作关键时刻触发 mark,
同时 IMU/相机自己记录自己的时间戳。
后期用 `wall_ts` 做 cross-correlation 校时:

```python
import json
import numpy as np

# 读 EMG 事件
emg_events = []  # 从 emg_xxx_events.csv 读

# 读 IMU 事件 (假设 IMU 也记录 wall_ts)
imu_events = []  # IMU 自己的格式

# 简单对齐: 用第一个 mark 做锚点, 后续事件按时间差对齐
t0_emg = emg_events[0]["wall_ts"]
t0_imu = imu_events[0]["wall_ts"]
offset = t0_imu - t0_emg   # IMU 比 EMG 早/晚 offset 秒

# 后续事件
for emg_e, imu_e in zip(emg_events, imu_events):
    diff = (imu_e["wall_ts"] - offset) - emg_e["wall_ts"]
    # diff 应该在 ±5ms 以内 (TCP 延迟)
```

更精确的对齐 (避免 TCP 抖动): 用 IMU 加速度信号和 EMG 包络做 cross-correlation。

---

## 6. 命令行参数

```
python collect_pure.py [选项]

选项:
  --no-input          不读键盘, 仅 TCP 触发 / Ctrl+C
  --tcp-port <port>   启动 TCP 触发服务器 (例: 8765)
  --session <name>    session 名 (默认: emg_<时间戳>)
  --customer <name>   客户名 (元数据)
  --scene <desc>      场景说明 (元数据)
```

### 常用组合

```bash
# 1. 客户交互模式 (常规)
python collect_pure.py

# 2. 自动化模式 (IMU/相机触发)
python collect_pure.py --no-input --tcp-port 8765

# 3. 批量采集 (预设 session 名)
python collect_pure.py --session task1_pick_apple --customer S001 --scene pick_apple

# 4. 自动化批量采集
python collect_pure.py --no-input --tcp-port 8765 --session task1_pick_apple --customer S001 --scene pick_apple
```

---

## 7. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `未发现任何串口` | USB 未插 / 驱动未装 | 重新插 USB, 装 CH340 驱动 |
| `[FAIL] 握手失败` | baudrate 不对 / 接线松 | 检查 2Mbps, 拧紧螺丝 |
| 数据全 0 | 电极没贴好 | 检查电极阻抗 |
| CH1 与 CH9 数据相同 | 通道复制 bug | 升级 ADS1299 固件 / 重新烧录 |
| 时间戳抖动大 | 后台负载高 | 关闭杀毒软件, 用 SSD |
| TCP 端口被占用 | 上一次没关干净 | 关闭占用 8765 的进程 |

---

## 8. 推荐工作流 (与 IMU/相机配合)

```
                    +-----------------+
   IMU 设备 ---->  | TCP 触发服务器  |
   相机     ---->  |  (本脚本)        |  ──> emg_<session>.npz
   操作员键盘 ---->  |                 |  ──> emg_<session>_events.csv
                    +-----------------+  ──> emg_<session>.zip

                  ↑ mark 事件同步打标 ↓
                  时间戳 wall_ts 对齐
```

**操作流程**:
1. 启动 IMU 采集器 (自己存 wall_ts)
2. 启动相机录制 (自己存 wall_ts)
3. **启动本采集器**: `python collect_pure.py --no-input --tcp-port 8765`
4. 客户开始任务
5. IMU/相机在每个动作的关键瞬间通过 TCP 发 `{"action":"mark","label":"..."}`
6. 任务结束, IMU/相机 发 `{"action":"stop"}`
7. 后处理: 用 `events.csv` 的 `wall_ts` 把 IMU/相机的标注映射到 EMG

---

## 9. 数据后处理建议

### 9.1 加载 + 滤波

```python
import numpy as np
from scipy.signal import butter, filtfilt

d = np.load("emg_S001_xxx.npz")
emg = d["data"]                  # (T, 32)
ts = d["timestamps"]

# 带通 20-450 Hz (EMG 标准)
b, a = butter(4, [20, 450], btype="band", fs=1000)
emg_filt = filtfilt(b, a, emg, axis=0)

# 包络 (RMS 窗口 100ms)
from scipy.signal import savgol_filter
window = 100
rms = np.sqrt(np.convolve(emg_filt[:, 0]**2,
                          np.ones(window)/window, mode="same"))
```

### 9.2 按事件切分

```python
import csv
events = []
with open("emg_S001_xxx_events.csv") as f:
    r = csv.DictReader(f)
    for row in r:
        events.append(row)

# 取 pick_apple 到 place 之间的区段
samples = d["sample_idx"]
for i, e in enumerate(events):
    if e["label"] == "pick_apple":
        start = int(e["sample_idx"])
    if e["label"] == "place" and i > 0:
        end = int(e["sample_idx"])
        seg = emg[start:end]   # 这段时间的 EMG
        print(f"动作段 {start}-{end}, {seg.shape[0]} 样本 ({seg.shape[0]/1000:.2f}s)")
```

### 9.3 训练数据组织

建议按 session / 客户 分子目录:
```
train_data/
├── S001/
│   ├── task1_pick_apple/
│   │   ├── emg_round1.npz
│   │   ├── emg_round1_events.csv
│   │   ├── emg_round2.npz
│   │   └── ...
│   └── task2_sort_size/
│       └── ...
└── S002/
    └── ...
```

---

## 10. 与 IMU/相机对齐的核心: 时间戳

本采集器每个样本记录一个 Unix 秒时间戳, 精度约 **±5 ms** (Linux/macOS) 或 **±15 ms** (Windows)。
这个精度对以下用途够用:
- ✅ IMU 加速度包络和 EMG RMS 的 cross-correlation
- ✅ 相机视频帧和 EMG 段的对齐 (假设视频 30fps, 即每帧 33ms)
- ✅ 多通道数据 (sEMG + IMU + 视频) 的全局同步

如果需要更高精度, 可以:
- 用 `time.monotonic_ns()` 替代 `time.time()` (本机内对齐)
- 用硬件同步信号 (BNO085 INT 引脚 → ADS1299 GPIO, 物理同步)

本采集器使用 `time.time()` 是为了**和 IMU/相机的 wall-clock 直接对齐** (跨设备), 这是更通用的方案。

---

## 联系与反馈

- 硬件问题 (LK-M1299 故障): 联系厂商
- 脚本问题: 看 `self_check.py` 排查, 或读源码

---

## 11. 多设备一键同步采集 (sEMG + IMU + 相机)

> 当你买到 IMU 和相机后，用这一章的方案。sEMG、IMU、相机各自延迟不同的问题，用**真实时间戳**完美解决。

### 11.1 核心设计思路

```
问题: sEMG 启动延迟 200ms, IMU 启动延迟 1.5s, 相机延迟 3s
      各设备运行时间戳各不相同，怎么对齐？

方案: 不对"启动时刻"做同步，对"真实时刻"做同步
      各设备记录的都是"真实世界几点几分几秒"
      对齐时按真实时间戳匹配，不管谁先启动
```

**关键：启动后等待 N 秒再正式记录，让各设备都充分热起来。**

```
10:00:00.000   sEMG 启动 (已进入缓冲)
10:00:01.200   IMU 启动
10:00:02.500   相机启动
10:00:07.000   倒计时结束, sync_base_ts = 10:00:07.000 (全局基准)
10:00:08.000   你开始做动作 A  ← 三个设备记录的都是 10:00:08.000
10:00:12.300   你做动作 B      ← 三个设备记录的都是 10:00:12.300
```

### 11.2 文件结构

```
customer_collector/
├── start_all.py        ← 一键启动入口 (Python)
├── config.json         ← 设备配置 (IP/端口/启用哪些)
├── collect_pure.py      ← sEMG 采集 (子进程)
├── imu_client.py       ← IMU 客户端 (子进程, 连接 ESP32+BNO085)
└── align.py            ← 后处理对齐工具

config.json 关键字段:
  prepare_delay_s: 5    ← 倒计时秒数 (客户准备时间)
  devices.semg.tcp_port: 8765
  devices.imu_left.tcp_port: 8766
  devices.imu_right.tcp_port: 8767
```

### 11.3 一键启动

```bash
# 编辑 config.json 填入你的设备信息:
#   - semg.ports: ["COM3", "COM4"]   ← 串口号
#   - imu_left/right.tcp_addr: 各 ESP32 连路由器后拿到的 IP (STA 模式, 烧录后看串口)
#   - WiFi 凭据在 firmware_imu/platformio.ini (不在 config.json, 左右手共用)

# 运行
python start_all.py
```

**自动流程**:

```
1. 并发启动: sEMG + IMU左腕 + IMU右腕 子进程
2. 等待所有设备 ping 就绪 (最多 20s)
3. 显示 5 秒倒计时 (客户准备)
4. 同时发 start 命令到所有设备 → 记录 sync_base_ts
5. 客户自由做动作 (任意分拣任务)
6. Ctrl+C 或 q 停止 → 同时发 stop 到所有设备
7. 保存数据到 session_YYYYMMDD_HHMMSS/ 目录
8. 打包 zip
```

**手动模式** (不用 start_all.py):

```bash
# 终端 1: sEMG 采集
python collect_pure.py --no-input --tcp-port 8765 --countdown 5

# 终端 2: IMU 左腕 (STA 模式: IP 填 ESP32 连路由器后串口看到的地址)
python imu_client.py --addr <左手IP> --port 8766 --position left

# 终端 3: IMU 右腕 (两个一起跑才验证了双手不冲突)
python imu_client.py --addr <右手IP> --port 8767 --position right

# 动作过程中, 在 sEMG 终端输入:
> m pick_apple   ← 标记动作
> m place
> q              ← 停止采集
```

### 11.4 数据输出

```
session_20260707_143022/
├── emg_session_20260707_143022.npz      ← 32 通道 sEMG + timestamps
├── emg_session_..._events.csv            ← 动作事件
├── emg_session_..._meta.json
├── imu_left_session_20260707_143022.npz  ← 6轴/9轴四元数 + 加速度 + 陀螺仪
├── imu_left_session_..._events.csv
├── imu_left_session_..._meta.json
├── imu_right_session_...npz
├── imu_right_session_..._events.csv
├── imu_right_session_..._meta.json
├── sync.json                              ← 同步基准 (全局 sync_base_ts)
└── session_20260707_143022.zip           ← 全部打包
```

### 11.5 对齐工具 (后处理)

```bash
# 查看各设备数据信息
python align.py emg.npz imu_left.npz imu_right.npz --info

# 按真实时间切分 (10.0s - 25.5s)
python align.py emg.npz imu_left.npz imu_right.npz --slice 10.0 25.5

# 自动对齐 (cross-correlation 找精确偏移)
python align.py emg.npz imu_left.npz --auto-align

# 按事件切分 (从 pick_apple 到 place)
python align.py emg.npz imu_left.npz --events events.csv
```

### 11.6 ESP32 固件

固件在 `firmware_imu/` (PlatformIO + Arduino C++, 已实现), 双 WiFi 模式:

```
默认 STA:  连家里路由器, DHCP 拿 IP (双手用, 解决 AP 冲突)
AP fallback: 板子自己当热点 (imu_left_ap / imu_right_ap, 单手离线调试)

做这几件事:
1. 连 WiFi (STA 连路由器 / AP 自建热点)
2. 作为 TCP 服务器监听 (左手 8766 / 右手 8767)
3. 通过 SHTP 协议读 BNO085 6轴/9轴四元数 + 加速度 + 陀螺仪
4. 100Hz 推 JSON: {"t":微秒,"q":[w,x,y,z],"q6":[w,x,y,z],"qa":3,"q6a":3,"a":[x,y,z],"g":[x,y,z]}
```

WiFi 凭据和静态 IP 在 `firmware_imu/src/main.cpp`; STA 连上后串口打印 IP, 填进 `config.json`。详见项目根 `README.md`。

### 11.6.1 四元数转腕部姿态轨迹

```bash
python orientation_trajectory.py imu_left_session_xxx.npz --side left --baseline-s 1
python orientation_trajectory.py imu_right_session_xxx.npz --side right --baseline-s 1
```

输出 `.npz`、`.csv` 和 `_meta.json`。新采集默认使用无磁力计的 6轴 `q6`,旧文件回退 9轴 `q`。
它给出的是腕带/前臂载体相对开头中立姿态的 3D 旋转,不计算 XYZ 位移。装入最终盒体后,
左右手各做一次已知方向动作,再用 `--axis-map` 固定载体轴到动作轴的映射。训练优先使用
`quat_relative_wxyz`;欧拉角只用于观察。

### 11.7 同步原理详解

| 对齐方式 | 精度 | 复杂度 | 适用场景 |
|---------|------|--------|---------|
| **真实时间戳 (本方案)** | ±5ms | 低 | sEMG + IMU + 相机，通用 |
| Cross-correlation 校准 | ±1ms | 中 | 有重复动作信号，可做后处理校准 |
| 硬件同步信号 | ±0.1ms | 高 | 医疗级精度，需要额外接线 |

**本方案选择理由**：IMU/相机/PC 之间无法用硬件同步线连接，真实时间戳是唯一可靠且简单的方案。

### 11.8 常见问题

| 问题 | 解决 |
|------|------|
| IMU 连接不上 | STA: 查 ESP32 串口是否 `[WiFi] (STA) 已连接` + IP 是否填对; ESP32 只认 2.4GHz/WPA2 |
| sync_base_ts 各设备不一致 | 允许 ±1s 偏差，后处理用 cross-correlation 校准 |
| 相机无时间戳 | 用 Python 的 time.time() 录制视频帧时打标 |
| 动作太快，对不齐 | 提高 IMU 采样率到 200Hz，减少延迟 |
| 数据量大，zip 太大 | 用 `np.savez_compressed` 已压缩，相机视频单独存 |
