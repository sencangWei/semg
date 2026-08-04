import struct
import numpy as np

from ads129x_cmd import ADS129X_Cmd
from ecg_filter import EcgFilter
from eeg_filter import EegFilter
from emg_filter import EmgFilter


# 数据解析类
class ADS129X_Data:
    def __init__(self):

        self.conn_state = 0x00
        self.rx_buff = []   # 数据包接收缓存区
        self.emg_raw_data = [[] for _ in range(16)]  # 初始化16通道原始数组缓存区, 仅使用前4个通道的脑电数据
        self.data_type = 'EMG'  # 信号类型  EEG EMG ECG
        self.sample_rate = 1000  # 采样率
        self.pga = 6            # 放大倍数（来自下位机反馈）
        self.ch_sw = 0          # 通道输入开关
        self.ch_en = 0xFFFF     # 通道使能位掩码
        self._duplicate_warned = False  # 通道重复数据警告标志
        self.eeg_filters = [EegFilter() for _ in range(16)]  # 脑电滤波器
        self.emg_filters = [EmgFilter() for _ in range(16)]  # 肌电滤波器
        self.ecg_filters = [EcgFilter() for _ in range(16)]  # 心电滤波器

    def clear(self):
        self.rx_buff = []
        self.emg_raw_data = [[] for _ in range(16)]
        self.sample_rate = 1000
        self.pga = 6
        self.ch_sw = 0
        self.ch_en = 0xFFFF
        self._duplicate_warned = False

    # 解包函数
    def parse_data(self, data_in):

        # hex_str = ' '.join(map(lambda b: f'{b:02X}', data_in))
        # print(hex_str)

        self.rx_buff += data_in
        data = self.rx_buff

        error_n = 0

        while len(data) > 6:

            # 帧头校验码
            head_xor = 0x00
            for b in data[1:5]:
                head_xor ^= b

            # 帧头校验
            if (data[0] == 0xAA) and (data[5] == head_xor):

                # 帧长度
                framLen = struct.unpack('<H', bytes(data[1:3]))[0]

                # 判断剩于数据长度是否大于帧长度
                if framLen <= len(data):
                    # 校验帧尾
                    if data[framLen - 1] == 0x55:
                        self.frame_unpack(bytes(data[0:framLen]))
                        del data[0:framLen]
                    else:
                        data.pop(0)
                        error_n+=1
                        # print("校验帧尾出错!")
                else:
                    break
            else:
                data.pop(0)
                error_n +=1
                # print("校验帧头出错!")

        self.rx_buff = data

        if error_n > 0 :
            print(f"error_n:{error_n}")

    def frame_unpack(self, data):

        # 命令码
        frameType = data[4] & 0x7F

        if frameType == ADS129X_Cmd.CMD_CONN_STATE:
            self.conn_state = data[6]

        if frameType == ADS129X_Cmd.CMD_SMAPLE_PAR:
            self.sample_rate = struct.unpack('<H', bytes(data[6:8]))[0]  # 采样率
            self.pga = data[8]  # 放大倍数
            self.ch_sw = data[9]  # 通道输入开关
            self.ch_en = struct.unpack('<H', bytes(data[10:12]))[0]  # 通道使能

        # 原始信号数据帧
        if frameType == ADS129X_Cmd.CMD_RAW_DATA:

            frameLen = struct.unpack('<H', bytes(data[1:3]))[0]
            # 自动识别实际通道数：frameLen=72 表示 16 通道，frameLen=40 表示 8 通道
            payload_bytes = frameLen - 8
            if payload_bytes % 64 == 0:
                ch_num = 16
            elif payload_bytes % 32 == 0:
                ch_num = 8
            else:
                print(f"[frame_unpack] 异常的 frameLen={frameLen}, payload={payload_bytes}，跳过该帧",
                      flush=True)
                return
            data_num = payload_bytes // (4 * ch_num)
            temp = data[6:(6 + data_num * (4*ch_num))]
            ch_datas = struct.unpack(f"<{ch_num * data_num}f", temp)

            # 调试：每 100 帧打印一次 frameLen 和 ch_num，方便定位 CH9-CH16 错位问题
            if not hasattr(self, '_dbg_frame_count'):
                self._dbg_frame_count = 0
            self._dbg_frame_count += 1
            if self._dbg_frame_count % 100 == 1 and data_num > 0:
                sample_ch0 = ch_datas[0] if len(ch_datas) > 0 else 0
                print(f"[frame_unpack] frameLen={frameLen}, ch_num={ch_num}, "
                      f"data_num={data_num}, sample_ch0={sample_ch0:.2f}",
                      flush=True)

            for i in range(data_num):

                # 获取同一时刻的采样的ch_num通道数据
                ch_data = ch_datas[i * ch_num : (i * ch_num + ch_num)]


                ch_data_f = []

                # 根据不同的信号类型选择，不同的滤波器
                if self.data_type == 'EEG':
                    for ch in range(ch_num):
                        data_temp = self.eeg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)
                elif self.data_type == 'EMG':
                    for ch in range(ch_num):
                        data_temp = self.emg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)
                elif self.data_type == 'ECG':
                    for ch in range(ch_num):
                        data_temp = self.ecg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)

                # 保存滤波后的数据
                for ch in range(ch_num):
                    self.emg_raw_data[ch].append(ch_data_f[ch])

            # 定期检查通道数据重复问题（仅 16 通道时检查）
            if (ch_num == 16 and
                    not self._duplicate_warned and
                    len(self.emg_raw_data[0]) >= 1000 and
                    len(self.emg_raw_data[8]) >= 1000):
                corr_ch0_8 = np.corrcoef(
                    np.array(self.emg_raw_data[0][:1000]),
                    np.array(self.emg_raw_data[8][:1000])
                )[0, 1]
                if corr_ch0_8 > 0.99:
                    print("[ADS129X_Data] 警告: CH1 与 CH9 数据相关性 > 0.99，"
                          "可能存在通道复制/接线问题，建议检查 ADS1299 固件或硬件接线！",
                          flush=True)
                    self._duplicate_warned = True
