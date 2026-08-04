import struct

# 指令集
class ADS129X_Cmd:

    CMD_SW_VERSION = 0x00  # 获取软件版本
    CMD_HW_VERSION = 0x01  # 获取硬件版本
    CMD_CONN_STATE = 0x03  # 连接状态更新
    CMD_START = 0x04 # 开始 / 停止采集指令
    CMD_SMAPLE_PAR = 0x05 # 采样参数配置
    CMD_RAW_DATA = 0x06 # 返回采样数据包

    # 采样率映射表
    SAMPLE_RATES = {
        "250sps": 250,
        "500sps": 500,
        "1000sps": 1000,
        "2000sps": 2000,
    }

    # EEG 量程映射表
    EEG_RANGES = {
        "±4.5V": 1,
        "±2.2V": 2,
        "±1.1V": 4,
        "±750mV": 6,
        "±560mV": 8,
        "±375mV": 12,
        "±186mV": 24
    }

    @staticmethod
    def cmd_data_pack(cmd, is_write, data):

        farme = []

        # 帧头
        farme.append(0xA5)

        # 帧长度
        farme += struct.pack('<H', len(data) + 8)

        # 地址
        farme.append(0x00)

        # 命令
        if is_write:
            farme.append((0x7F&cmd))  # 写指令
        else:
            farme.append(0x80&cmd)  # 读指令

        # 帧头校验码
        head_xor = 0x00
        for b in farme[1:]:
            head_xor ^= b

        farme.append(head_xor)

        # 数据内容
        farme += data

        # 数据校验
        data_xor = 0x00
        for b in data:
            data_xor ^= b

        farme.append(data_xor)

        # 帧尾
        farme.append(0x5A)

        # # 使用map函数和format格式化
        # hex_str = ' '.join(map(lambda b: f'{b:02X}', farme))
        # print(f'cmd send: {hex_str}')

        return farme

    # 连接状态指令
    @staticmethod
    def conn_status_update_cmd(state):
        return ADS129X_Cmd.cmd_data_pack(ADS129X_Cmd.CMD_CONN_STATE, True, [state])

    # 开始采集指令
    @staticmethod
    def start_collect_cmd():
        return ADS129X_Cmd.cmd_data_pack(ADS129X_Cmd.CMD_START, True, [0x01])

    # 停止采集指令
    @staticmethod
    def stop_collect_cmd():
        return ADS129X_Cmd.cmd_data_pack(ADS129X_Cmd.CMD_START, True, [0x00])

    # 设置采样参数
    @staticmethod
    def set_sample_par_cmd(rate_str,eeg_range_str, eeg_ch_en):
        """
        设置采样参数

        参数:
        rate_str: 采样率字符串，如 "1000sps"
        range_str: 量程字符串，如 "±4.5V"
        ch_en: 通道使能字节，每个bit对应一个通道(1-8)

        返回:
        字节数组形式的指令
        """
        # 解析采样率
        eeg_rate_val = ADS129X_Cmd.SAMPLE_RATES.get(rate_str, 500)  # 默认500sps

        # 解析量程
        eeg_range_val = ADS129X_Cmd.EEG_RANGES.get(eeg_range_str, 6)  # 默认±400mV

        # 构建数据
        data = list(struct.pack('<H', eeg_rate_val))  # 采样率  2字节小端
        data.append(eeg_range_val)  # EEG 量程 1字节
        data.append(0x00)  # 1字节保留
        data +=  list(struct.pack('<H', eeg_ch_en))    # EEG通道使能 1字节

        return ADS129X_Cmd.cmd_data_pack(ADS129X_Cmd.CMD_SMAPLE_PAR, True, data)

