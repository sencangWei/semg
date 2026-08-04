import asyncio
import csv
import sys
import threading
import time
import datetime

import numpy as np
from PyQt5.QtCore import QObject, QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QLabel, QVBoxLayout, QWidget,
                             QMainWindow, QHBoxLayout, QSizePolicy, QSpacerItem,
                             QComboBox, QPushButton, QGroupBox, QCheckBox,
                             QFileDialog, QMessageBox, QGridLayout, QLineEdit)
from PyQt5.QtNetwork import QTcpServer, QTcpSocket, QHostAddress

import pyqtgraph as pg
from pyqtgraph import PlotWidget

from ads129x_cmd import ADS129X_Cmd
from ads129x_data import ADS129X_Data

# 设置pyqtgraph全局选项
pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class MainWindow(QMainWindow, QObject):
    data_received = pyqtSignal(bytes)  # 添加自定义信号，用于传递接收到的数据

    def __init__(self):
        super().__init__()
        self.initUI()

        # 其他初始化
        self.ads129x_data = ADS129X_Data()
        self.channel_states = [True] * 16
        self.display_length = 5000
        self.data_buffer = []  # 接收数据的临时缓冲（保留以兼容原有逻辑）
        self.buffer_lock = threading.Lock()  # 缓冲区锁（保留）
        self.paused = False
        self.collect_sate = False

        # TCP 相关对象
        self.tcp_server = None
        self.tcp_socket = None

        # 连接数据接收信号
        self.data_received.connect(self.on_data_received)

        # 定时器用于刷新图形
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(200)

    def initUI(self):
        # 通道颜色（保持不变）
        self.channel_colors = [
            (0, 114, 189), (217, 83, 25), (237, 177, 32), (126, 47, 142),
            (119, 172, 48), (77, 190, 238), (162, 20, 47), (76, 76, 76),
            (0, 114, 189), (217, 83, 25), (237, 177, 32), (126, 47, 142),
            (119, 172, 48), (77, 190, 238), (162, 20, 47), (76, 76, 76)
        ]

        self.setWindowTitle('脑电心电数据采集软件')
        self.setGeometry(100, 100, 1400, 800)

        # 左侧布局
        left_vbox = QVBoxLayout()
        left_vbox.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        left_vbox.addSpacerItem(QSpacerItem(60, 0, QSizePolicy.Fixed, QSizePolicy.Fixed))
        left_vbox.setSpacing(15)

        # --- TCP 服务器配置区域 ---
        hbox_tcp = QHBoxLayout()
        hbox_tcp.addWidget(QLabel("IP地址:"))
        self.ip_edit = QLineEdit("192.168.8.188")
        hbox_tcp.addWidget(self.ip_edit)
        hbox_tcp.addWidget(QLabel("端口:"))
        self.port_edit = QLineEdit("8888")
        hbox_tcp.addWidget(self.port_edit)
        left_vbox.addLayout(hbox_tcp)

        # 操作按钮
        self.btn_start_server = QPushButton('启动服务器', self)
        self.btn_start_server.clicked.connect(self.toggle_server)
        self.btn_start = QPushButton('开始采集', self)
        self.btn_start.clicked.connect(self.start_collect)
        self.btn_start.setEnabled(False)
        self.btn_export_csv = QPushButton('导出数据', self)
        self.btn_export_csv.clicked.connect(self.export_csv)
        self.btn_export_csv.setEnabled(False)
        self.btn_auto_scale = QPushButton('自动缩放', self)
        self.btn_auto_scale.clicked.connect(self.auto_scale)
        self.btn_pause = QPushButton('暂停显示', self)
        self.btn_pause.clicked.connect(self.toggle_pause)

        left_vbox.addWidget(self.btn_start_server)
        left_vbox.addWidget(self.btn_start)
        left_vbox.addWidget(self.btn_export_csv)
        left_vbox.addWidget(self.btn_auto_scale)
        left_vbox.addWidget(self.btn_pause)

        # 通道选择区域
        group_channel = QGroupBox("通道选择")
        grid_channel = QGridLayout()
        self.channel_buttons = []
        for i in range(16):
            row = i // 4
            col = i % 4
            chk = QCheckBox(f"CH{i+1}")
            chk.setChecked(True)
            chk.toggled.connect(lambda state, idx=i: self.toggle_channel(state, idx))
            grid_channel.addWidget(chk, row, col)
            self.channel_buttons.append(chk)
        group_channel.setLayout(grid_channel)
        left_vbox.addWidget(group_channel)

        # 采样参数设置
        group_smaple_param = QGroupBox("采样参数设置")
        layout_smaple_param = QVBoxLayout()
        hbox_rate = QHBoxLayout()
        hbox_rate.addWidget(QLabel("采样率:"))
        self.rate_combo = QComboBox()
        self.rate_combo.addItems(["250sps", "500sps", "1000sps", "2000sps"])
        self.rate_combo.setCurrentText("2000sps")
        hbox_rate.addWidget(self.rate_combo)
        layout_smaple_param.addLayout(hbox_rate)

        hbox_eeg_range = QHBoxLayout()
        hbox_eeg_range.addWidget(QLabel("脑电量程:"))
        self.eeg_range_combo = QComboBox()
        self.eeg_range_combo.addItems(["±1.1V", "±750mV", "±560mV", "±375mV", "±186mV"])
        self.eeg_range_combo.setCurrentText("±375mV")
        hbox_eeg_range.addWidget(self.eeg_range_combo)
        layout_smaple_param.addLayout(hbox_eeg_range)
        group_smaple_param.setLayout(layout_smaple_param)
        left_vbox.addWidget(group_smaple_param)

        left_container = QWidget()
        left_container.setFixedWidth(220)
        left_container.setLayout(left_vbox)

        # 右侧绘图区域
        right_vbox = QVBoxLayout()
        self.graph_widget_eeg = PlotWidget()
        self.graph_widget_eeg.setBackground('w')
        self.graph_widget_eeg.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget_eeg.setLabel('left', '电压', units='μV')
        self.graph_widget_eeg.setLabel('bottom', '采样点')
        self.graph_widget_eeg.setTitle('14通道肌电信号')
        self.graph_widget_eeg.setYRange(0, 1700)
        self.graph_widget_eeg.addLegend()

        self.plots_eeg = []
        for i in range(16):
            color = self.channel_colors[i]
            pen = pg.mkPen(color=color, width=1)
            plot = self.graph_widget_eeg.plot([], [], pen=pen, name=f'CH{i+1}', antialias=True)
            self.plots_eeg.append(plot)
        right_vbox.addWidget(self.graph_widget_eeg)

        # 主布局
        hbox_main = QHBoxLayout()
        hbox_main.addWidget(left_container)
        hbox_main.addLayout(right_vbox, 1)

        widget_central = QWidget()
        widget_central.setLayout(hbox_main)
        self.setCentralWidget(widget_central)

    # ---------- TCP 服务器管理 ----------
    def toggle_server(self):
        """启动/停止 TCP 服务器"""
        if self.btn_start_server.text() == "启动服务器":
            self.start_server()
        else:
            self.stop_server()

    def start_server(self):
        """启动 TCP 服务器，开始监听"""
        ip = self.ip_edit.text()
        try:
            port = int(self.port_edit.text())
        except ValueError:
            QMessageBox.critical(self, "错误", "端口号必须是整数")
            return

        self.tcp_server = QTcpServer(self)
        if self.tcp_server.listen(QHostAddress(ip), port):
            self.btn_start_server.setText("停止服务器")
            self.btn_start.setEnabled(True)
            self.tcp_server.newConnection.connect(self.on_new_connection)
            print(f"TCP 服务器启动在 {ip}:{port}")
        else:
            QMessageBox.critical(self, "错误", f"无法启动服务器: {self.tcp_server.errorString()}")
            self.tcp_server = None

    def stop_server(self):
        """停止 TCP 服务器，断开所有连接"""
        if self.tcp_socket:
            self.tcp_socket.disconnectFromHost()
            self.tcp_socket = None
        if self.tcp_server:
            self.tcp_server.close()
            self.tcp_server = None
        self.btn_start_server.setText("启动服务器")
        self.btn_start.setEnabled(False)
        self.btn_export_csv.setEnabled(False)
        print("TCP 服务器已停止")

    def on_new_connection(self):
        """处理新客户端连接"""
        if self.tcp_socket:
            # 已有连接，拒绝新连接
            new_socket = self.tcp_server.nextPendingConnection()
            new_socket.disconnectFromHost()
            print("已有客户端连接，拒绝新连接")
            return
        self.tcp_socket = self.tcp_server.nextPendingConnection()
        self.tcp_socket.readyRead.connect(self.on_tcp_data_received)
        self.tcp_socket.disconnected.connect(self.on_tcp_disconnected)
        print(f"客户端已连接: {self.tcp_socket.peerAddress().toString()}:{self.tcp_socket.peerPort()}")

    def on_tcp_data_received(self):
        """接收 TCP 数据，并通过信号传递"""
        data = self.tcp_socket.readAll()
        if data:
            self.data_received.emit(data.data())

    def on_tcp_disconnected(self):
        """客户端断开连接"""
        print("客户端断开连接")
        if self.tcp_socket:
            self.tcp_socket.deleteLater()
            self.tcp_socket = None
        # 如果正在采集中，可选择自动停止或保留状态，这里保留采集状态但禁止发送命令
        # 用户需手动重新连接后重新开始采集

    # ---------- 数据接收与解析 ----------
    def on_data_received(self, data):
        """处理接收到的数据（与原串口版本相同）"""
        # 将数据放入缓冲区（保留原有逻辑）
        self.data_buffer.append(data)
        if self.data_buffer:
            all_data = b''.join(self.data_buffer)
            self.data_buffer.clear()
            if all_data:
                self.ads129x_data.parse_data(all_data)

    # ---------- 命令发送（通过 TCP） ----------
    def apply_sample_params(self):
        """应用采样参数设置，通过 TCP 发送命令"""
        rate_str = self.rate_combo.currentText()
        eeg_range_str = self.eeg_range_combo.currentText()
        eeg_ch_en = 0x0000
        for i in range(16):
            if self.channel_buttons[i].isChecked():
                eeg_ch_en |= (1 << i)
        try:
            cmd = ADS129X_Cmd.set_sample_par_cmd(rate_str, eeg_range_str, eeg_ch_en)
            self.tcp_socket.write(bytes(cmd))
            # if self.tcp_socket and self.tcp_socket.state() == QTcpSocket.ConnectedState:
            #     self.tcp_socket.write(bytes(cmd))
            # else:
            #     QMessageBox.critical(self, "错误", "TCP 连接未建立，无法发送参数")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送采样参数失败: {e}")

    def start_collect(self):
        """开始/停止采集"""
        if self.btn_start.text() == "开始采集":
            if self.tcp_socket and self.tcp_socket.state() == QTcpSocket.ConnectedState:
                self.btn_start.setText("停止采集")
                self.btn_export_csv.setEnabled(True)
                self.ads129x_data.clear()
                self.data_buffer.clear()
                # 禁用参数修改控件
                self.rate_combo.setEnabled(False)
                self.eeg_range_combo.setEnabled(False)
                for chk in self.channel_buttons:
                    chk.setEnabled(False)

                # self.apply_sample_params()
                # time.sleep(1)

                try:
                    self.collect_sate = True
                    self.tcp_socket.write(bytes(ADS129X_Cmd.start_collect_cmd()))
                except Exception as e:
                    QMessageBox.critical(self, "错误", f"发送开始命令失败: {e}")
                    self.btn_export_csv.setEnabled(False)
            else:
                QMessageBox.critical(self, "错误", "TCP 连接未建立，无法开始采集")
        else:
            # 停止采集
            self.collect_sate = False
            self.btn_start.setText("开始采集")
            self.rate_combo.setEnabled(True)
            self.eeg_range_combo.setEnabled(True)
            for chk in self.channel_buttons:
                chk.setEnabled(True)

            if self.tcp_socket and self.tcp_socket.state() == QTcpSocket.ConnectedState:
                try:
                    self.tcp_socket.write(bytes(ADS129X_Cmd.stop_collect_cmd()))
                except Exception as e:
                    QMessageBox.critical(self, "错误", f"发送停止命令失败: {e}")

    # ---------- 图形更新及其他功能（与原串口版本相同） ----------
    def toggle_channel(self, state, channel_idx):
        self.channel_states[channel_idx] = bool(state)
        self.plots_eeg[channel_idx].setVisible(state)

    def update_plot(self):
        if self.paused or not self.collect_sate:
            return

        if len(self.ads129x_data.emg_raw_data[0]) > 0:
            for i in range(16):
                if self.channel_states[i] and len(self.ads129x_data.emg_raw_data[i]) > 0:
                    channel_data = self.ads129x_data.emg_raw_data[i]
                    data_length = len(channel_data)
                    start_idx = max(0, data_length - self.display_length)
                    display_data = channel_data[start_idx:]
                    offset = i * 100 + 100
                    y_data = np.array(display_data) + offset
                    x_data = np.arange(len(y_data)) + start_idx
                    self.plots_eeg[i].setData(x_data, y_data, clear=True)

            data_length = len(self.ads129x_data.emg_raw_data[0])
            x_min = max(0, data_length - self.display_length)
            x_max = data_length
            self.graph_widget_eeg.setXRange(x_min, x_max)

    def auto_scale(self):
        self.graph_widget_eeg.autoRange()

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.setText("继续显示" if self.paused else "暂停显示")

    def export_csv(self):
        if len(self.ads129x_data.emg_raw_data) == 0:
            QMessageBox.information(None, "提示", "采样数据缓存为空，请先采集数据!", QMessageBox.Ok)
            return

        now = datetime.datetime.now()
        default_name = now.strftime("%Y%m%d_%H%M%S") + "_" + str(self.ads129x_data.sample_rate) + "sps"
        filepath, _ = QFileDialog.getSaveFileName(None, "保存CSV文件", default_name, "CSV文件 (*.csv);;所有文件 (*)")
        if not filepath:
            return

        num_eeg = min(16, len(self.ads129x_data.emg_raw_data))
        if num_eeg == 0:
            QMessageBox.information(None, "提示", "没有有效EEG通道数据可导出！", QMessageBox.Ok)
            return

        headers = [f"EEG_CH{i+1}" for i in range(num_eeg)]
        channels = self.ads129x_data.emg_raw_data[:num_eeg]
        max_len = max(len(ch) for ch in channels) if channels else 0

        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for i in range(max_len):
                row = []
                for ch_data in channels:
                    if i < len(ch_data):
                        value = ch_data[i]
                        row.append(f"{value:.3f}" if isinstance(value, float) else str(value))
                    else:
                        row.append("")
                writer.writerow(row)

        QMessageBox.information(None, "提示", "导出成功！", QMessageBox.Ok)

    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        # 停止采集
        if self.btn_start.text() == "停止采集":
            self.start_collect()
        # 停止服务器
        if self.btn_start_server.text() == "停止服务器":
            self.stop_server()
        self.update_timer.stop()
        event.accept()


# ---------- 异步主函数（保持与原框架一致） ----------
async def main():
    app = QApplication(sys.argv)
    from qasync import QEventLoop
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = MainWindow()
    window.show()

    with loop:
        await loop.run_forever()


if __name__ == '__main__':
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")