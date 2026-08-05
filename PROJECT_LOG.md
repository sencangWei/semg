# semg_repo 项目日志

> 双臂 sEMG + 双腕 IMU 采集系统（客户交付版）。
> GitHub: https://github.com/sencangWei/semg

---

## 当前状态

**阶段**: 仓库已整理推送，README 已完善，后续开发主要在 Ubuntu 同步 skill 配置后恢复。

---

## 仓库结构

```
semg_repo/
├── firmware/        # ESP32-S3 + BNO085 IMU 固件（WiFi/TCP 推流）
├── collector/       # Python 采集上位机
└── resources/       # 硬件资料、原理图、说明书
```

---

## 已完成（2026-08-04 ~ 2026-08-05）

### 合并与推送
- [x] 将三个独立项目（firmware、customer_collector、resources）合并为一个仓库。
- [x] 写好 `.gitignore`，排除 `.pio` / `.venv` / 数据 / 静态库 / 3D 模型等。
- [x] `git init` + commit + force push 到 https://github.com/sencangWei/semg。
- [x] 删除误建的 `firmware_imu` 和 `customer_collector` 空仓库。

### README
- [x] 扩展 README.md：硬件参数、接线、采集流程、数据格式、常见问题、文档索引。

---

## 下一步

- [ ] 等 Ubuntu i3 mini PC 开机后，同步 `~/.claude/skills/` 和 memory 配置。
- [ ] 客户现场采集前，跑 `collector/self_check.py` 验证环境。
- [ ] 如需 IMU 联调，确认左右手 STA 静态 IP（.49 / .50）或切 AP 模式。

---

## 关键参数

| 项目 | 值 | 说明 |
|------|-----|------|
| ESP32-S3 Flash | 4MB | 必须 `board_upload.flash_size = 4MB` |
| IMU 左手 IP | 192.168.113.49 | STA 静态 IP |
| IMU 右手 IP | 192.168.113.50 | STA 静态 IP |
| IMU 左手端口 | 8766 | TCP 推流 |
| IMU 右手端口 | 8767 | TCP 推流 |
| sEMG 采样率 | 1000 Hz | 默认 |
| sEMG 通道数 | 16/32 | 单臂 16ch，双臂 32ch |

---

## 已知坑

- ESP32-S3 实际 4MB Flash，devkitc-1 默认 8MB 会启动崩溃。
- Windows 下 `-D` 宏传字符串会被转义破坏，WiFi 凭据直接写 `firmware/src/main.cpp` 顶部。
- CH1 与 CH9 相关系数 > 0.99 可能是固件 bug。

---

**更新日期**: 2026-08-05
