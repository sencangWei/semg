import struct

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
        self.eeg_filters = [EegFilter() for _ in range(16)]  # 脑电滤波器
        self.emg_filters = [EmgFilter() for _ in range(16)]  # 肌电滤波器
        self.ecg_filters = [EcgFilter() for _ in range(16)]  # 心电滤波器

    def clear(self):
        self.rx_buff = []
        self.emg_raw_data = [[] for _ in range(16)]

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
            self.sample_rate  = struct.unpack('<H', bytes(data[6:8]))[0]  # 采样率
            pga = data[8] # 放大倍数
            ch_sw = data[9]  # 通道输入开关
            ch_en = struct.unpack('<H', bytes(data[10:12]))[0]  # 通道使能

        # 原始信号数据帧
        if frameType == ADS129X_Cmd.CMD_RAW_DATA:

            frameLen = struct.unpack('<H', bytes(data[1:3]))[0]
            ch_num = 16
            data_num = int((frameLen - 8) / (4*ch_num))
            temp = data[6:(6 + data_num * (4*ch_num))]
            ch_datas = struct.unpack(f"<{ch_num * data_num}f", temp)

            for i in range(data_num):

                # 获取同一时刻的采样的ch_num通道数据
                ch_data = ch_datas[i * ch_num : (i * ch_num + ch_num)]


                ch_data_f = []

                # 根据不同的信号类型选择，不同的滤波器
                if self.data_type == 'EEG':
                    for ch in range(16):
                        data_temp = self.eeg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)
                elif self.data_type == 'EMG':
                    for ch in range(16):
                        data_temp = self.emg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)
                elif self.data_type == 'ECG':
                    for ch in range(16):
                        data_temp = self.ecg_filters[ch].process_sample(self.sample_rate, ch_data[ch])
                        ch_data_f.append(data_temp)

                # 保存滤波后的脑电数据
                for ch in range(16):
                    self.emg_raw_data[ch].append(ch_data_f[ch])
