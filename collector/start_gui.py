"""LignoEMG-16CH 多设备采集系统 - 基于 data_collector.py 架构.

参考 data_collector.py 的成熟实现，重写 start_gui.py：
  - SerialWorker + ADS129X_Data 读取设备数据
  - pyqtgraph 实时波形显示 (16/32 通道垂直偏移)
  - 自动连续记录，无手动标注
  - 可选接入 IMU + 相机

工作流程:
  1. 刷新串口 → 选择端口 → 打开设备
  2. 开始采集 (带准备倒计时)
  3. 系统自动记录所有数据
  4. 停止 → 自动保存

用法:
  python start_gui.py
"""

import sys
import os
import time
import datetime
import json
import threading
import zipfile
import csv
import argparse
from pathlib import Path
from collections import deque

import numpy as np

import pyqtgraph as pg
from pyqtgraph import PlotWidget
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QGroupBox,
                             QGridLayout, QComboBox, QSpinBox, QLineEdit,
                             QMessageBox, QTextEdit)
from PyQt5.QtGui import QFont
import serial
from serial.tools import list_ports

pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

_HERE = Path(__file__).parent.resolve()
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE / "core"))

from ads129x_cmd import ADS129X_Cmd
from ads129x_data import ADS129X_Data

N_CHANNELS = 16  # 每块 LK-M1299 的通道数
SAMPLE_RATE = 1000

CHANNEL_COLORS = [
    (0, 114, 189), (217, 83, 25), (237, 177, 32), (126, 47, 142),
    (119, 172, 48), (77, 190, 238), (162, 20, 47), (128, 128, 128),
    (255, 127, 0), (0, 158, 115), (214, 39, 40), (140, 86, 75),
    (44, 160, 44), (255, 152, 213), (148, 103, 189), (31, 119, 180),
]

# IMU / 相机可选
try:
    from imu_client import IMUClient
    HAS_IMU = True
except ImportError:
    HAS_IMU = False

try:
    from camera_client import CameraClient
    HAS_CAMERA = True
except ImportError:
    HAS_CAMERA = False


# =============================================================================
# 核心：SerialWorker（直接来自 data_collector.py，稳定可用）
# =============================================================================
class SerialWorker(QThread):
    """串口数据读取线程。直接参考 data_collector.py."""
    data_received = pyqtSignal(bytes)

    def __init__(self, serial_port=None):
        super().__init__()
        self.serial_port = serial_port
        self.running = False
        self.lock = threading.Lock()

    def set_serial_port(self, serial_port):
        with self.lock:
            self.serial_port = serial_port

    def run(self):
        self.running = True
        while self.running:
            with self.lock:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        available = self.serial_port.in_waiting
                        if available > 0:
                            data = self.serial_port.read(available)
                            if data:
                                self.data_received.emit(data)
                        else:
                            self.msleep(5)
                    except Exception as e:
                        print(f"串口读取错误: {e}")
                        self.msleep(10)
                else:
                    self.msleep(10)

    def stop(self):
        self.running = False
        self.wait(1000)


# =============================================================================
# 双设备管理（简化版，参考 data_collector 单臂模式 + 双臂扩展）
# =============================================================================
class DualDeviceManager:
    """管理 1-2 台 LK-M1299，参考 data_collector.py 单设备模式."""

    def __init__(self):
        self.devices = []  # list of _DeviceHandle

    def open(self, ports: list) -> bool:
        """打开 1 或 2 个端口."""
        ports = list(dict.fromkeys(ports))[:2]
        for idx, port in enumerate(ports):
            arm = 'L' if idx == 0 else 'R'
            try:
                ser = serial.Serial(
                    port=port, baudrate=2_000_000,
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
                print(f"[ERR] 打开 {port} 失败: {e}")
                return False

            ads = ADS129X_Data()
            worker = SerialWorker(ser)
            dev = _DeviceHandle(
                port_name=port, arm_label=arm, arm_idx=idx,
                serial_port=ser, ads=ads, worker=worker,
            )
            self.devices.append(dev)
        return True

    def start_workers(self):
        """启动读取线程，发送采集命令."""
        for dev in self.devices:
            dev.worker.data_received.connect(dev._on_data)
            dev.worker.setPriority(QThread.HighPriority)
            dev.worker.start()

        for dev in self.devices:
            try:
                cmd = ADS129X_Cmd.set_sample_par_cmd("1000sps", "±375mV", 0xFFFF)
                dev.serial_port.write(bytes(cmd))
                time.sleep(0.1)
                dev.serial_port.write(bytes(ADS129X_Cmd.start_collect_cmd()))
                time.sleep(0.1)
            except Exception as e:
                print(f"[ERR] 发送采集命令失败 ({dev.port_name}): {e}")

    def stop_workers(self):
        """停止采集并关闭."""
        for dev in self.devices:
            try:
                dev.serial_port.write(bytes(ADS129X_Cmd.stop_collect_cmd()))
            except Exception:
                pass
        time.sleep(0.2)
        for dev in self.devices:
            dev.worker.stop()
        for dev in self.devices:
            try:
                dev.serial_port.close()
            except Exception:
                pass

    @property
    def n_devices(self) -> int:
        return len(self.devices)

    @property
    def n_total_channels(self) -> int:
        return self.n_devices * N_CHANNELS

    def channel_names(self) -> list:
        names = []
        for dev in self.devices:
            for ch in range(N_CHANNELS):
                names.append(f'CH{ch+1}_{dev.arm_label}')
        return names

    def clear_all(self):
        for dev in self.devices:
            dev.ads.clear()

    def poll_all(self):
        """返回 (all_samples, all_times)，从所有设备收集最新样本."""
        all_samples = []
        all_times = []
        now = time.time()

        for dev in self.devices:
            if not dev.ads.emg_raw_data or len(dev.ads.emg_raw_data[0]) == 0:
                continue
            n = len(dev.ads.emg_raw_data[0])
            for i in range(n):
                row = [dev.ads.emg_raw_data[ch][i] if ch < len(dev.ads.emg_raw_data) and i < len(dev.ads.emg_raw_data[ch]) else 0.0
                       for ch in range(N_CHANNELS)]
                all_samples.append(row)
                all_times.append(now - (n - 1 - i) / SAMPLE_RATE)
        return all_samples, all_times


class _DeviceHandle:
    def __init__(self, port_name, arm_label, arm_idx, serial_port, ads, worker):
        self.port_name = port_name
        self.arm_label = arm_label
        self.arm_idx = arm_idx
        self.serial_port = serial_port
        self.ads = ads
        self.worker = worker

    def _on_data(self, data: bytes):
        self.ads.parse_data(data)


# =============================================================================
# GUI 主窗口
# =============================================================================
class CollectorGUI(QMainWindow):

    def __init__(self):
        super().__init__()
        self.dev_mgr = DualDeviceManager()
        self.channel_states = [True] * 32
        self.display_length = 4000

        # 采集状态
        self.collecting = False
        self.start_wall = None
        self.stop_wall = None
        self.sync_base_ts = None

        # 数据缓冲区
        self._buffer_lock = threading.Lock()
        self._sample_buffer = deque(maxlen=120 * SAMPLE_RATE)  # 120秒缓存
        self._time_buffer = deque(maxlen=120 * SAMPLE_RATE)

        # IMU / 相机
        self.imu_left = None
        self.imu_right = None
        self.cam_left = None
        self.cam_right = None

        # 原始数据（用于保存）
        self._raw_samples = []
        self._raw_times = []

        self.initUI()

    def initUI(self):
        self.setWindowTitle('LignoEMG-16CH 多设备采集系统')
        self.setGeometry(100, 100, 2000, 1050)

        # ==================== 左侧面板 ====================
        left_vbox = QVBoxLayout()
        left_vbox.setAlignment(Qt.AlignTop)
        left_vbox.setSpacing(10)

        # 连接状态
        self.label_conn = QLabel("连接状态: 未连接")
        self.label_conn.setStyleSheet("font-weight: bold; color: red; font-size: 13px;")
        left_vbox.addWidget(self.label_conn)

        # 端口选择
        port_group = QGroupBox("设备端口")
        gl = QGridLayout()
        gl.addWidget(QLabel("左臂端口:"), 0, 0)
        self.port_combo_L = QComboBox()
        gl.addWidget(self.port_combo_L, 0, 1)
        gl.addWidget(QLabel("右臂端口:"), 1, 0)
        self.port_combo_R = QComboBox()
        gl.addWidget(self.port_combo_R, 1, 1)
        port_group.setLayout(gl)
        left_vbox.addWidget(port_group)

        btn_row = QHBoxLayout()
        self.btn_refresh = QPushButton('刷新')
        self.btn_refresh.clicked.connect(self.refresh_ports)
        btn_row.addWidget(self.btn_refresh)
        self.btn_open = QPushButton('打开串口')
        self.btn_open.clicked.connect(self.open_serial)
        btn_row.addWidget(self.btn_open)
        left_vbox.addLayout(btn_row)

        # 采集控制
        ctrl_group = QGroupBox("采集控制")
        cv = QVBoxLayout()

        hbox_prep = QHBoxLayout()
        hbox_prep.addWidget(QLabel("准备时间:"))
        self.spin_prep = QSpinBox()
        self.spin_prep.setRange(0, 30)
        self.spin_prep.setValue(3)
        self.spin_prep.setSuffix(" 秒")
        hbox_prep.addWidget(self.spin_prep)
        cv.addLayout(hbox_prep)

        self.btn_start = QPushButton('开始采集')
        self.btn_start.clicked.connect(self.start_collect)
        self.btn_start.setEnabled(False)
        self.btn_start.setStyleSheet(
            "background: #27ae60; color: white; font-weight: bold; padding: 10px; font-size: 14px;"
        )
        cv.addWidget(self.btn_start)

        self.btn_stop = QPushButton('停止采集')
        self.btn_stop.clicked.connect(self.stop_collect)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            "background: #c0392b; color: white; font-weight: bold; padding: 10px; font-size: 14px;"
        )
        cv.addWidget(self.btn_stop)

        ctrl_group.setLayout(cv)
        left_vbox.addWidget(ctrl_group)

        # IMU 状态
        if HAS_IMU:
            imu_group = QGroupBox("IMU 设备")
            igl = QGridLayout()
            self.label_imu_L = QLabel("左腕: 未连接")
            self.label_imu_L.setStyleSheet("color: gray;")
            igl.addWidget(self.label_imu_L, 0, 0, 1, 2)
            self.label_imu_R = QLabel("右腕: 未连接")
            self.label_imu_R.setStyleSheet("color: gray;")
            igl.addWidget(self.label_imu_R, 1, 0, 1, 2)
            imu_group.setLayout(igl)
            left_vbox.addWidget(imu_group)

        # 相机状态
        if HAS_CAMERA:
            cam_group = QGroupBox("相机设备")
            cgl = QGridLayout()
            self.label_cam_L = QLabel("左相机: 未连接")
            self.label_cam_L.setStyleSheet("color: gray;")
            cgl.addWidget(self.label_cam_L, 0, 0, 1, 2)
            self.label_cam_R = QLabel("右相机: 未连接")
            self.label_cam_R.setStyleSheet("color: gray;")
            cgl.addWidget(self.label_cam_R, 1, 0, 1, 2)
            cam_group.setLayout(cgl)
            left_vbox.addWidget(cam_group)

        # 保存
        self.btn_save = QPushButton('保存数据')
        self.btn_save.clicked.connect(self.save_data)
        self.btn_save.setStyleSheet(
            "background: #8e44ad; color: white; font-weight: bold; padding: 10px; font-size: 14px;"
        )
        left_vbox.addWidget(self.btn_save)

        # 日志
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet("font-family: Consolas; font-size: 11px;")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        left_vbox.addWidget(log_group)

        left_vbox.addStretch()

        left_cw = QWidget()
        left_cw.setFixedWidth(270)
        left_cw.setLayout(left_vbox)

        # ==================== 右侧面板 ====================
        right_vbox = QVBoxLayout()

        # 状态横幅
        self.status_label = QLabel('待机 (请先打开串口)')
        self.status_label.setStyleSheet(
            'font-size: 20px; font-weight: bold; color: white; '
            'background: #2c3e50; border-radius: 8px; padding: 16px; '
            'qproperty-alignment: AlignCenter;'
        )
        self.status_label.setMinimumHeight(60)
        right_vbox.addWidget(self.status_label)

        # 同步信息
        self.sync_label = QLabel('sync_base_ts: --')
        self.sync_label.setStyleSheet(
            'font-size: 11px; color: #7f8c8d; padding: 2px; '
            'qproperty-alignment: AlignCenter;'
        )
        right_vbox.addWidget(self.sync_label)

        sep = QLabel('— 实时 EMG 波形 —')
        sep.setStyleSheet('color: #7f8c8d; padding: 2px;')
        sep.setAlignment(Qt.AlignCenter)
        right_vbox.addWidget(sep)

        # 实时波形 (pyqtgraph，参考 data_collector.py 的实现)
        self.graph_widget = PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget.setLabel('left', '电压', units='uV')
        self.graph_widget.setLabel('bottom', '时间', units='s')
        self.graph_widget.setTitle('EMG 实时信号')
        self.graph_widget.addLegend()

        self.plots = []
        for i in range(32):  # 最大 32 通道
            color = CHANNEL_COLORS[i % N_CHANNELS]
            if i >= N_CHANNELS:
                color = tuple(max(0, c - 50) for c in color)
            pen = pg.mkPen(color=color, width=1)
            label = f'CH{i+1}'
            plot = self.graph_widget.plot([], [], pen=pen, name=label, antialias=True)
            self.plots.append(plot)

        right_vbox.addWidget(self.graph_widget)

        # 主布局
        hbox = QHBoxLayout()
        hbox.addWidget(left_cw)
        hbox.addLayout(right_vbox, 1)

        cw = QWidget()
        cw.setLayout(hbox)
        self.setCentralWidget(cw)

        # 定时器
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(100)

        self.refresh_ports()
        self._log("系统启动，等待连接设备...")

    def _log(self, msg):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    # ----------------------------------------------------------------
    # 端口操作（直接来自 data_collector.py）
    # ----------------------------------------------------------------
    def refresh_ports(self):
        ports = sorted(list_ports.comports())
        if sys.platform != "win32":
            ports = [p for p in ports if not p.device.startswith("/dev/ttyS")]
        port_list = [f"{p.device}" for p in ports] if ports else ["未检测到串口"]
        self.port_combo_L.blockSignals(True)
        self.port_combo_R.blockSignals(True)
        self.port_combo_L.clear()
        self.port_combo_R.clear()
        self.port_combo_L.addItems(port_list)
        self.port_combo_R.addItems(port_list)
        self.port_combo_L.blockSignals(False)
        self.port_combo_R.blockSignals(False)
        self.port_combo_L.setEnabled(bool(ports))
        self.port_combo_R.setEnabled(bool(ports))
        self.btn_open.setEnabled(bool(ports))

    def open_serial(self):
        if self.btn_open.text() == "关闭串口":
            self._do_close()
            return

        port_L = self.port_combo_L.currentText()
        port_R = self.port_combo_R.currentText()

        if port_L == "未检测到串口":
            QMessageBox.critical(self, "错误", "未检测到任何串口")
            return

        ports = [port_L]
        if port_R and port_R != port_L and port_R != "未检测到串口":
            ports.append(port_R)

        if not self.dev_mgr.open(ports):
            QMessageBox.critical(self, "错误", f"无法打开端口 {ports}")
            return

        self.dev_mgr.start_workers()
        time.sleep(0.3)

        n_dev = self.dev_mgr.n_devices
        arm_str = "双臂" if n_dev == 2 else "单臂"
        ch_str = self.dev_mgr.n_total_channels

        self.btn_open.setText("关闭串口")
        self.label_conn.setText(f"已连接: {arm_str} ({ch_str}ch)")
        self.label_conn.setStyleSheet("font-weight: bold; color: green; font-size: 13px;")
        self.btn_start.setEnabled(True)
        self._log(f"设备已连接: {ports}")

        # IMU
        self._connect_imu()
        # 相机
        self._connect_camera()

    def _do_close(self):
        if self.collecting:
            self.stop_collect()
        self.dev_mgr.stop_workers()
        self.label_conn.setText("连接状态: 未连接")
        self.label_conn.setStyleSheet("font-weight: bold; color: red; font-size: 13px;")
        self.btn_open.setText("打开串口")
        self.btn_start.setEnabled(False)
        self._log("设备已关闭")

    def serial_send(self, cmd):
        for dev in self.dev_mgr.devices:
            try:
                if dev.serial_port.is_open:
                    dev.serial_port.write(cmd)
            except Exception as e:
                print(f"[ERR] 发送命令失败: {e}")

    # ----------------------------------------------------------------
    # IMU / 相机
    # ----------------------------------------------------------------
    def _connect_imu(self):
        if not HAS_IMU:
            return
        self._log("IMU 模块未加载 (imu_client.py)")

    def _connect_camera(self):
        if not HAS_CAMERA:
            return
        self._log("相机模块未加载 (camera_client.py)")

    # ----------------------------------------------------------------
    # 采集控制（参考 data_collector.py）
    # ----------------------------------------------------------------
    def start_collect(self):
        if self.dev_mgr.n_devices == 0:
            QMessageBox.warning(self, "提示", "请先打开串口!")
            return

        self.collecting = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_open.setEnabled(False)

        # 清缓冲区
        self.dev_mgr.clear_all()
        with self._buffer_lock:
            self._sample_buffer.clear()
            self._time_buffer.clear()
        self._raw_samples = []
        self._raw_times = []

        # 准备倒计时
        prep = self.spin_prep.value()
        if prep > 0:
            self.status_label.setText(f"准备中... {prep}秒后开始")
            self.status_label.setStyleSheet(
                'font-size: 18px; font-weight: bold; color: white; '
                'background: #e67e22; border-radius: 8px; padding: 16px; '
                'qproperty-alignment: AlignCenter;'
            )
            self.status_label.repaint()
            QApplication.processEvents()
            for i in range(prep, 0, -1):
                self.status_label.setText(f"准备中... {i}秒")
                self.status_label.repaint()
                QApplication.processEvents()
                time.sleep(1.0)

        # 正式采集
        with self._buffer_lock:
            self.start_wall = time.time()
            self.stop_wall = None
            self.sync_base_ts = self.start_wall

        n_ch = self.dev_mgr.n_total_channels
        arm_str = "双臂" if self.dev_mgr.n_devices == 2 else "单臂"
        self.sync_label.setText(f'sync_base_ts = {self.sync_base_ts:.6f}')
        self.sync_label.setStyleSheet('font-size: 11px; color: #27ae60; font-weight: bold; padding: 2px; qproperty-alignment: AlignCenter;')

        self.status_label.setText(f'采集中  |  {arm_str} {n_ch}ch')
        self.status_label.setStyleSheet(
            'font-size: 18px; font-weight: bold; color: white; '
            'background: #27ae60; border-radius: 8px; padding: 16px; '
            'qproperty-alignment: AlignCenter;'
        )
        self._log(f"采集开始: {arm_str} {n_ch}ch, sync={self.sync_base_ts:.3f}")

    def stop_collect(self):
        self.collecting = False
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_open.setEnabled(True)

        self.stop_wall = time.time()
        n_buf = len(self._sample_buffer)
        elapsed = self.stop_wall - self.start_wall if self.start_wall else 0

        n_dev = self.dev_mgr.n_devices
        arm_str = "双臂" if n_dev == 2 else "单臂"
        self.label_conn.setText(f"已连接: {arm_str} ({self.dev_mgr.n_total_channels}ch)")
        self.label_conn.setStyleSheet("font-weight: bold; color: green; font-size: 13px;")

        self.status_label.setText(f'已停止  |  {n_buf} 样本  |  {elapsed:.1f}s')
        self.status_label.setStyleSheet(
            'font-size: 18px; font-weight: bold; color: white; '
            'background: #34495e; border-radius: 8px; padding: 16px; '
            'qproperty-alignment: AlignCenter;'
        )
        self._log(f"采集停止: {n_buf}样本, {elapsed:.1f}s")

    # ----------------------------------------------------------------
    # 绘图（参考 data_collector.py 的 pyqtgraph 实现）
    # ----------------------------------------------------------------
    def update_plot(self):
        # 实时拉取数据（参考 data_collector.py 的方式）
        if self.collecting and self.dev_mgr.n_devices > 0:
            self._poll_devices()

        # 非采集状态：检查连接
        if not self.collecting:
            connected = any(
                dev.ads.emg_raw_data and len(dev.ads.emg_raw_data[0]) > 0
                for dev in self.dev_mgr.devices
            ) if self.dev_mgr.devices else False
            if connected:
                n_dev = self.dev_mgr.n_devices
                arm_str = "双臂" if n_dev == 2 else "单臂"
                self.label_conn.setText(f"已连接: {arm_str} ({self.dev_mgr.n_total_channels}ch)")
                self.label_conn.setStyleSheet("font-weight: bold; color: green; font-size: 13px;")
            return

        # 绘制波形（直接参考 data_collector.py）
        with self._buffer_lock:
            buf_len = len(self._sample_buffer)
            if buf_len < 10:
                return
            n_show = min(self.display_length, buf_len)
            recent = list(self._sample_buffer)[-n_show:]

        n_ch = self.dev_mgr.n_total_channels
        n_buf = len(self._sample_buffer)
        elapsed = time.time() - self.sync_base_ts if self.sync_base_ts else 0
        arm_str = "双臂" if self.dev_mgr.n_devices == 2 else "单臂"
        self.status_label.setText(f'采集中  |  {n_buf}样本  |  {elapsed:.1f}s  |  {arm_str} {n_ch}ch')

        x_data = np.arange(len(recent))

        for i in range(n_ch):
            if self.channel_states[i]:
                ch_vals = [s[i] if i < len(s) else 0.0 for s in recent]
                offset = i * 1000 + 1000
                y_data = np.array(ch_vals) + offset
                self.plots[i].setData(x_data, y_data, clear=True)

        if len(recent) > 0:
            self.graph_widget.setXRange(0, len(recent))

    def _poll_devices(self):
        """从设备缓冲区收集最新样本."""
        for dev in self.dev_mgr.devices:
            if not dev.ads.emg_raw_data or len(dev.ads.emg_raw_data[0]) == 0:
                continue
            n = len(dev.ads.emg_raw_data[0])
            now = time.time()
            for i in range(n):
                row = []
                for ch in range(N_CHANNELS):
                    if ch < len(dev.ads.emg_raw_data) and i < len(dev.ads.emg_raw_data[ch]):
                        row.append(dev.ads.emg_raw_data[ch][i])
                    else:
                        row.append(0.0)
                with self._buffer_lock:
                    self._sample_buffer.append(row)
                    self._time_buffer.append(now - (n - 1 - i) / SAMPLE_RATE)
            # 清空已读数据
            for ch in range(N_CHANNELS):
                if ch < len(dev.ads.emg_raw_data):
                    dev.ads.emg_raw_data[ch] = []

    # ----------------------------------------------------------------
    # 保存（参考 data_collector.py 的保存逻辑）
    # ----------------------------------------------------------------
    def save_data(self):
        with self._buffer_lock:
            n_buf = len(self._sample_buffer)

        if n_buf == 0:
            QMessageBox.information(self, "提示", "没有采集到数据!")
            return

        ts = datetime.datetime.now().strftime('%Y%mdd_%H%M%S')
        session_name = f"session_{ts}"

        default_dir = os.path.join(os.path.expanduser('~'), 'Desktop')
        save_dir = default_dir
        os.makedirs(save_dir, exist_ok=True)

        folder = f"{session_name}"
        full_dir = os.path.join(save_dir, folder)
        os.makedirs(full_dir, exist_ok=True)

        # 采集数据
        with self._buffer_lock:
            data = np.array(self._sample_buffer, dtype=np.float32)
            times = np.array(self._time_buffer, dtype=np.float64)

        elapsed = times - (self.sync_base_ts or times[0])
        channel_names = self.dev_mgr.channel_names()
        arm_labels = [dev.arm_label for dev in self.dev_mgr.devices]

        # 1. NPZ
        npz_path = os.path.join(full_dir, 'raw_emg.npz')
        np.savez_compressed(
            npz_path,
            data=data,
            timestamps=times,
            elapsed_since_sync=elapsed,
            sample_idx=np.arange(data.shape[0], dtype=np.int32),
            sample_rate=SAMPLE_RATE,
            channel_names=np.array(channel_names),
            n_channels=self.dev_mgr.n_total_channels,
            arm_labels=np.array(arm_labels),
            start_wall=np.float64(self.start_wall or 0.0),
            stop_wall=np.float64(self.stop_wall or 0.0),
            sync_base_ts=np.float64(self.sync_base_ts or 0.0),
        )

        # 2. CSV
        csv_path = os.path.join(full_dir, 'emg_data.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time'] + [f'CH{i+1}' for i in range(data.shape[1])])
            for t, row in zip(times, data):
                writer.writerow([f"{t:.6f}"] + [f"{v:.2f}" for v in row])

        # 3. metadata.json
        meta = {
            "collection_time": ts,
            "session": session_name,
            "sample_rate_hz": SAMPLE_RATE,
            "n_channels": self.dev_mgr.n_total_channels,
            "channel_names": channel_names,
            "n_devices": self.dev_mgr.n_devices,
            "device_arms": arm_labels,
            "n_samples": int(data.shape[0]),
            "duration_s": float(times[-1] - times[0]) if len(times) > 1 else 0,
            "start_wall": self.start_wall,
            "stop_wall": self.stop_wall,
            "sync_base_ts": self.sync_base_ts,
        }
        meta_path = os.path.join(full_dir, 'metadata.json')
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        # 4. sync.json
        sync = {
            "global_sync_base_ts": self.sync_base_ts,
            "global_sync_base_iso": datetime.datetime.fromtimestamp(self.sync_base_ts).isoformat() if self.sync_base_ts else None,
        }
        sync_path = os.path.join(full_dir, 'sync.json')
        with open(sync_path, 'w', encoding='utf-8') as f:
            json.dump(sync, f, ensure_ascii=False, indent=2)

        # 5. 打包 zip
        zip_path = os.path.join(save_dir, f"{folder}.zip")
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            for fname in ['raw_emg.npz', 'emg_data.csv', 'metadata.json', 'sync.json']:
                fpath = os.path.join(full_dir, fname)
                if os.path.exists(fpath):
                    zf.write(fpath, arcname=fname)

        self._log(f"已保存: {full_dir}/  ({n_buf}样本)")
        QMessageBox.information(
            self, "保存完成",
            f"数据已保存:\n{full_dir}\n\n"
            f"sync_base_ts = {self.sync_base_ts:.6f}\n"
            f"{data.shape[0]} 样本, {meta['duration_s']:.1f}s\n\n"
            f"打包: {zip_path}"
        )

    def closeEvent(self, event):
        if self.collecting:
            self.stop_collect()
        self.dev_mgr.stop_workers()
        event.accept()


# =============================================================================
# 入口
# =============================================================================
def main():
    print("=" * 60)
    print("  LignoEMG-16CH 多设备采集系统")
    print("  基于 data_collector.py 架构")
    print("=" * 60)

    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = CollectorGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
