import asyncio
import csv
import sys
import time
import datetime
import threading

import numpy as np
from PyQt5.QtCore import QObject, QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QLabel, QVBoxLayout, QWidget,
                             QMainWindow, QHBoxLayout, QSizePolicy, QSpacerItem,
                             QComboBox, QPushButton, QGroupBox, QCheckBox,
                             QFileDialog, QMessageBox, QProgressDialog, QGridLayout)

import pyqtgraph as pg
from pyqtgraph import PlotWidget
import serial
from serial.tools import list_ports

from ads129x_cmd import ADS129X_Cmd
from ads129x_data import ADS129X_Data

# 设置pyqtgraph全局选项
pg.setConfigOptions(antialias=True)  # 开启抗锯齿
pg.setConfigOption('background', 'w')  # 白色背景
pg.setConfigOption('foreground', 'k')  # 黑色前景

# 串口读取工作线程
class SerialWorker(QThread):
    """串口数据读取工作线程"""
    data_received = pyqtSignal(bytes)  # 信号：接收到数据

    def __init__(self, serial_port=None):
        super().__init__()
        self.serial_port = serial_port
        self.running = False
        self.lock = threading.Lock()

    def set_serial_port(self, serial_port):
        """设置串口对象（线程安全）"""
        with self.lock:
            self.serial_port = serial_port

    def run(self):
        """线程主循环"""
        self.running = True

        while self.running:
            with self.lock:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        # 检查是否有数据可读
                        available = self.serial_port.in_waiting
                        if available > 0:
                            # 读取数据
                            data = self.serial_port.read(available)
                            # print(len(data))
                            if data:
                                self.data_received.emit(data)
                        else:
                            # 没有数据时稍微休息，避免占用全部CPU
                            self.msleep(5)
                            # print('msleep(50)')
                    except Exception as e:
                        error_msg = f"串口读取错误: {e}"
                        print(error_msg)
                        self.msleep(10)  # 发生错误时休息一下
                else:
                    self.msleep(10)  # 串口未打开时休息

    def stop(self):
        """停止线程"""
        self.running = False
        self.wait(1000)  # 等待1秒

# 主窗口
class MainWindow(QMainWindow, QObject):

    def __init__(self):
        super().__init__()

        # 初始化变量
        self.ads129x_data = ADS129X_Data()  # 数据缓存及解析
        self.serial_port = None  # 串口对象
        self.serial_worker = None  # 串口工作线程
        self.channel_states = [True] * 16  # 通道显示状态
        self.display_length = 5000  # 显示点数（5秒数据，假设采样率1000Hz）
        self.paused = False  # 暂停状态
        self.collect_sate = False

        # 初始化UI
        self.initUI()

    # 界面初始化
    def initUI(self):

        # channel_colors
        self.channel_colors = [
            (0, 114, 189),  # 蓝色
            (217, 83, 25),  # 橙色
            (237, 177, 32),  # 金黄色
            (126, 47, 142),  # 紫色
            (119, 172, 48),  # 草绿色
            (77, 190, 238),  # 天青色
            (162, 20, 47),  # 深红色
            (128, 128, 128),  # 中灰色
            (255, 127, 0),  # 琥珀橙
            (0, 158, 115),  # 翠绿色
            (214, 39, 40),  # 亮红
            (140, 86, 75),  # 棕褐色
            (44, 160, 44),  # 鲜绿色
            (255, 152, 213),  # 粉红色
            (148, 103, 189),  # 淡紫色
            (31, 119, 180)  # 深蓝色
        ]

        # 界面的标题
        self.setWindowTitle('LK-M1299-16CH')

        # 界面的位置和大小的设置
        self.setGeometry(100, 100, 1400, 800)

        # 左侧垂直布局
        left_vbox = QVBoxLayout()
        left_vbox.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        left_vbox.addSpacerItem(QSpacerItem(60, 0, QSizePolicy.Fixed, QSizePolicy.Fixed))
        left_vbox.setSpacing(15)

        # 连接状态
        self.label_conn_state = QLabel("连接状态: 未连接")

        # 创建下拉框/串口选择
        self.port_combo = QComboBox(self)
        self.port_combo.addItems([])
        hbox_uart = QHBoxLayout()
        hbox_uart.addWidget(QLabel("选择串口"))
        hbox_uart.addWidget(self.port_combo)

        # 创建操作按钮
        self.btn_refresh = QPushButton('刷新串口', self)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_open = QPushButton('打开串口', self)
        self.btn_open.clicked.connect(self.open_serial)
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

        # 创建通道选择区域（使用网格布局，1行4列）
        group_channel = QGroupBox("通道选择")
        grid_channel = QGridLayout()

        self.channel_buttons = []
        for i in range(16):
            # 计算行和列
            row = i // 4  # 0或1
            col = i % 4  # 0,1,2,3
            chk = QCheckBox(f"CH{i + 1}", self)  # 仅使用数字
            chk.setChecked(True)
            chk.toggled.connect(lambda state, idx=i: self.toggle_channel(state, idx))
            grid_channel.addWidget(chk, row, col)
            self.channel_buttons.append(chk)

        # 设置列之间的间距
        grid_channel.setHorizontalSpacing(4)
        grid_channel.setVerticalSpacing(2)

        # 设置边距
        grid_channel.setContentsMargins(10, 10, 10, 10)

        group_channel.setLayout(grid_channel)

        # 添加采样参数设置区域
        group_smaple_param = QGroupBox("采样参数设置")
        layout_smaple_param = QVBoxLayout()

        # 采样率选择
        hbox_rate = QHBoxLayout()
        hbox_rate.addWidget(QLabel("采样率:"))
        self.rate_combo = QComboBox(self)
        self.rate_combo.addItems(["250sps", "500sps", "1000sps", "2000sps"])
        self.rate_combo.setCurrentText("1000sps")
        hbox_rate.addWidget(self.rate_combo)
        layout_smaple_param.addLayout(hbox_rate)

        # 量程选择
        hbox_eeg_range = QHBoxLayout()
        hbox_eeg_range.addWidget(QLabel("脑电量程:"))
        self.eeg_range_combo = QComboBox(self)
        self.eeg_range_combo.addItems(["±1.1V", "±750mV", "±560mV", "±375mV", "±186mV"])
        self.eeg_range_combo.setCurrentText("±375mV")
        hbox_eeg_range.addWidget(self.eeg_range_combo)

        layout_smaple_param.addLayout(hbox_eeg_range)
        group_smaple_param.setLayout(layout_smaple_param)

        left_vbox.addWidget(self.label_conn_state)
        left_vbox.addLayout(hbox_uart)
        left_vbox.addWidget(self.btn_refresh)
        left_vbox.addWidget(self.btn_open)
        left_vbox.addWidget(self.btn_start)
        left_vbox.addWidget(self.btn_export_csv)
        left_vbox.addWidget(self.btn_auto_scale)
        left_vbox.addWidget(self.btn_pause)
        left_vbox.addWidget(group_channel)
        left_vbox.addWidget(group_smaple_param)

        # 创建左侧容器并设置固定宽度
        left_container = QWidget()
        left_container.setFixedWidth(220)  # 固定宽度320像素
        left_container.setLayout(left_vbox)

        # 右侧垂直布局 - 使用pyqtgraph
        right_vbox = QVBoxLayout()

        # 创建脑电 pyqtgraph图形窗口
        self.graph_widget_eeg = PlotWidget()
        self.graph_widget_eeg.setBackground('w')
        self.graph_widget_eeg.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget_eeg.setLabel('left', '电压', units='μV')
        self.graph_widget_eeg.setLabel('bottom', '采样点')
        self.graph_widget_eeg.setTitle('16通道信号')
        if self.ads129x_data.data_type == 'EEG':
            self.graph_widget_eeg.setYRange(0, 1700)  # 初始Y轴范围
        else:
            self.graph_widget_eeg.setYRange(0, 17000) # 初始Y轴范围

        # 添加图例
        self.graph_widget_eeg.addLegend()

        # 创建16条曲线
        self.plots_eeg = []
        for i in range(16):
            color = self.channel_colors[i]
            pen = pg.mkPen(color=color, width=1)
            plot = self.graph_widget_eeg.plot(
                [], [],
                pen=pen,
                name=f'CH{i + 1}',
                antialias=True
            )
            self.plots_eeg.append(plot)

        right_vbox.addWidget(self.graph_widget_eeg)

        # 创建整体界面的水平布局
        hbox_main = QHBoxLayout()
        hbox_main.addWidget(left_container)  # 左侧固定宽度，不拉伸
        hbox_main.addLayout(right_vbox, 1)  # 右侧可拉伸，拉伸因子为1

        # 创建一个中心小部件并设置布局
        widget_central = QWidget()
        widget_central.setLayout(hbox_main)
        self.setCentralWidget(widget_central)

        # 定时器设置
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(1000)

        # 刷新串口列表
        self.refresh_ports()

    # 刷新串口
    def refresh_ports(self):
        """刷新可用的串口列表"""
        current_selection = self.port_combo.currentText()
        self.port_combo.clear()

        ports = list_ports.comports()

        if not ports:
            self.port_combo.addItem("未检测到串口")
            self.port_combo.setEnabled(False)
            self.btn_open.setEnabled(False)
        else:
            for port in sorted(ports):
                self.port_combo.addItem(f"{port.device} - {port.description}")
            self.port_combo.setEnabled(True)
            self.btn_open.setEnabled(True)

            if current_selection in [self.port_combo.itemText(i) for i in range(self.port_combo.count())]:
                self.port_combo.setCurrentText(current_selection)
            elif ports:
                self.port_combo.setCurrentIndex(0)

    # 打开串口
    def open_serial(self):
        """打开或关闭串口"""
        if self.btn_open.text() == "打开串口":
            port_name = self.port_combo.currentText().split(" - ")[0]
            try:
                self.serial_port = serial.Serial(
                    port=port_name,
                    baudrate=2000000,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1,
                )
                self.serial_port.set_buffer_size(rx_size=102400, tx_size=65536)
                self.serial_port.setRTS(False)  # RTS 无效
                self.serial_port.setDTR(False)  # DTR 无效

                if self.serial_port.isOpen():
                    self.btn_open.setText("关闭串口")
                    self.label_conn_state.setText("连接状态: 未连接")
                    self.btn_start.setEnabled(True)

                    # 创建并启动串口工作线程
                    self.serial_worker = SerialWorker(self.serial_port)
                    self.serial_worker.data_received.connect(self.on_data_received)
                    self.serial_worker.setPriority(QThread.HighPriority)
                    self.serial_worker.start()

                    print(f"打开串口成功: {port_name}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"打开串口失败: {e}")
                print(f"打开串口失败: {e}")
        else:
            if self.btn_start.text() == "停止采集":
                self.start_collect()

            # 关闭串口
            self.label_conn_state.setText("连接状态: 未连接")
            self.btn_open.setText("打开串口")
            self.btn_start.setText("开始采集")
            self.btn_start.setEnabled(False)
            self.btn_export_csv.setEnabled(False)  # 关闭串口时禁用导出按钮

            # 停止工作线程
            if self.serial_worker:
                self.serial_worker.stop()
                self.serial_worker = None

            # 关闭串口
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None

            self.ads129x_data.conn_state = 0x00

    # 串口发送指令
    def serial_send_cmd(self,cmd):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd)

                # 打印发送指令
                hex_str = ' '.join(map(lambda b: f'{b:02X}', cmd))
                print(f'cmd send: {hex_str}')

            except Exception as e:
                QMessageBox.critical(self, "错误", f"发送停止命令失败: {e}")

    # 处理接收到的串口数据
    def on_data_received(self, data):
        self.ads129x_data.parse_data(data)

    # 发送配置采样参数指令
    def apply_sample_params(self):

        # 获取选择的采样率
        rate_str = self.rate_combo.currentText()

        # 获取选择的EEG量程
        eeg_range_str = self.eeg_range_combo.currentText()

        # 计算通道使能字节
        eeg_ch_en = 0x0000
        for i in range(16):
            if self.channel_buttons[i].isChecked():
                eeg_ch_en |= (1 << i)

        # 生成设置采样参数的指令
        cmd = ADS129X_Cmd.set_sample_par_cmd(rate_str, eeg_range_str, eeg_ch_en)

        # 发送指令
        self.serial_send_cmd(bytes(cmd))

    # 发送开始/停止采集指令
    def start_collect(self):
        if self.btn_start.text() == "开始采集":
            if self.serial_port and self.serial_port.is_open:
                self.btn_start.setText("停止采集")
                self.btn_export_csv.setEnabled(True)  # 启用导出按钮
                self.ads129x_data.clear()
                # self.paused = False

                self.rate_combo.setEnabled(False)
                self.eeg_range_combo.setEnabled(False)
                for i in range(16):
                    self.channel_buttons[i].setEnabled(False)

                # 发送配置采样参数指令
                self.apply_sample_params()
                time.sleep(0.3)

                self.collect_sate = True

                # 发送开始采集命令
                self.serial_send_cmd(bytes(ADS129X_Cmd.start_collect_cmd()))
                self.update_timer.setInterval(200)
        else:
            self.collect_sate = False
            self.btn_start.setText("开始采集")
            self.rate_combo.setEnabled(True)
            self.eeg_range_combo.setEnabled(True)
            for i in range(16):
                self.channel_buttons[i].setEnabled(True)
            self.serial_send_cmd(bytes(ADS129X_Cmd.stop_collect_cmd()))
            self.update_timer.setInterval(1000)

    # 通道显示开关
    def toggle_channel(self, state, channel_idx):
        self.channel_states[channel_idx] = bool(state)
        self.plots_eeg[channel_idx].setVisible(state)

    # 解析数据包，并更新绘图
    def update_plot(self):

        if self.collect_sate == False:
            # 发送保持连接状态指令
            self.serial_send_cmd(bytes(ADS129X_Cmd.conn_status_update_cmd(0x02)))
            if self.ads129x_data.conn_state > 0x00:
                self.label_conn_state.setText("连接状态: 已连接")
            else:
                self.label_conn_state.setText("连接状态: 未连接")
            self.update_timer.setInterval(1000)
            return

        #  暂停显示判断
        if self.paused:
            return

        # 更新脑电绘图数据
        if len(self.ads129x_data.emg_raw_data[0]) > 0:

            # 更新每个通道的绘图数据
            for i in range(16):
                if self.channel_states[i] and len(self.ads129x_data.emg_raw_data[i]) > 0:
                    # 获取最新的数据
                    channel_data = self.ads129x_data.emg_raw_data[i]
                    data_length = len(channel_data)

                    # 只显示最新的display_length个点
                    start_idx = max(0, data_length - self.display_length)
                    display_data = channel_data[start_idx:]

                    # 添加偏移量以便区分通道
                    offset = 0
                    if self.ads129x_data.data_type == 'EEG' :
                       offset = i * 100 + 100
                    else:
                       offset = i * 1000 + 1000
                    y_data = np.array(display_data) + offset

                    # 生成X轴数据
                    x_data = np.arange(len(y_data)) + start_idx

                    # 更新曲线
                    self.plots_eeg[i].setData(x_data, y_data, clear=True)

            # 自动调整X轴范围
            if len(self.ads129x_data.emg_raw_data[0]) > 0:
                data_length = len(self.ads129x_data.emg_raw_data[0])
                x_min = max(0, data_length - self.display_length)
                x_max = data_length
                self.graph_widget_eeg.setXRange(x_min, x_max)

    # Y轴自适应缩放
    def auto_scale(self):
        """自动缩放"""
        self.graph_widget_eeg.autoRange()
        self.graph_widget_ecg.autoRange()

    # 暂停/继续实时显示
    def toggle_pause(self):
        """切换暂停状态"""
        self.paused = not self.paused
        if self.paused:
            self.btn_pause.setText("继续显示")
        else:
            self.btn_pause.setText("暂停显示")

    # 导出csv数据
    def export_csv(self):
        if len(self.ads129x_data.emg_raw_data) == 0:
            QMessageBox.information(None, "提示", "采样数据缓存为空，请先采集数据!", QMessageBox.Ok)
            return

        now = datetime.datetime.now()
        default_name = now.strftime("%Y%m%d_%H%M%S") + "_" + str(self.ads129x_data.sample_rate) + "sps"
        filepath, _ = QFileDialog.getSaveFileName(
            None,
            "保存CSV文件",
            default_name,
            "CSV文件 (*.csv);;所有文件 (*)"
        )

        if filepath == "":
            return

        # 1. 确定要导出的 EEG 通道（最多16个，但不超过实际通道数）
        num_eeg = min(16, len(self.ads129x_data.emg_raw_data))
        if num_eeg == 0:
            QMessageBox.information(None, "提示", "没有有效EEG通道数据可导出！", QMessageBox.Ok)
            return

        # 3. 构建表头
        headers = [f"EEG_CH{i + 1}" for i in range(num_eeg)]

        # 4. 收集所有要导出的通道数据
        channels = self.ads129x_data.emg_raw_data[:num_eeg]  # EEG 前 num_eeg 个通道

        # 5. 计算最大数据长度
        max_len = max(len(ch) for ch in channels) if channels else 0

        # 6. 写入 CSV 文件
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

            for i in range(max_len):
                row = []
                for ch_data in channels:
                    if i < len(ch_data):
                        value = ch_data[i]
                        # 如果是浮点数则保留三位小数，否则转字符串（空值处理）
                        row.append(f"{value:.3f}" if isinstance(value, float) else str(value))
                    else:
                        row.append("")  # 缺失单元格留空
                writer.writerow(row)

        QMessageBox.information(None, "提示", "导出成功！", QMessageBox.Ok)

    # 关闭窗体
    def closeEvent(self, event):
        """关闭窗口时的清理工作"""
        # 停止采集
        if self.btn_start.text() == "停止采集":
            self.start_collect()

        # 关闭串口
        if self.btn_open.text() == "关闭串口":
            self.open_serial()

        # 停止定时器
        self.update_timer.stop()

        event.accept()

# 异步主函数
async def main():
    app = QApplication(sys.argv)
    # 使用 qasync 将 asyncio 事件循环与 PyQt 集成
    from qasync import QEventLoop
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    # 创建主窗口
    main_window = MainWindow()
    main_window.show()

    # 启动事件循环
    with loop:
        await loop.run_forever()


if __name__ == '__main__':
    # 设置高DPI支持
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