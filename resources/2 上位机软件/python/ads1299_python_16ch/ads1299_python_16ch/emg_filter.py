"""
肌电信号(EMG)滤波器
提供针对不同采样率的肌电信号滤波处理
"""
from iir_filter import IirFilter


class EmgFilter:
    """
    肌电信号(EMG)滤波器主类
    针对不同采样率自动配置高通滤波器和50Hz梳状工频陷波器
    """

    def __init__(self):
        """初始化滤波器组，支持500、1000、2000、4000 Hz采样率"""
        self._filters_by_sampling_rate = {}
        self._init_filters_for_250()
        self._init_filters_for_500()
        self._init_filters_for_1000()
        self._init_filters_for_2000()
        self._init_filters_for_4000()

    @staticmethod
    def _generate_comb_filter_coefficients(start_val, end_val, order):
        """
        生成梳状滤波器系数

        Args:
            start_val: 起始系数值
            end_val: 结束系数值
            order: 滤波器阶数

        Returns:
            系数列表，长度为 order+1，中间填充零
        """
        coeffs = [start_val] + [0.0] * (order - 1) + [end_val]
        return coeffs

    def _init_filters_for_250(self):
        """初始化500Hz采样率的滤波器组"""
        # 18Hz高通滤波器（4阶）
        b_hp = [0.550405843485670,-2.20162337394268,3.30243506091402,-2.20162337394268,0.550405843485670]
        a_hp = [1,-2.82237942110333,3.11264838091628,-1.56851671780884,0.302948975942266]

        # 50Hz梳状工频陷波器（10阶）
        b_notch =  [0.968979151360103,-0.598862049930573,0.968979151360103]
        a_notch = [1,-0.598862049930573	,0.937958302720205]

        filters = [IirFilter(b_hp, a_hp), IirFilter(b_notch, a_notch)]
        self._filters_by_sampling_rate[250] = filters

    def _init_filters_for_500(self):
        """初始化500Hz采样率的滤波器组"""
        # 18Hz高通滤波器（4阶）
        b_hp = [0.743579590743563, -2.97431836297425, 4.46147754446138,
                -2.97431836297425, 0.743579590743563]
        a_hp = [1, -3.40952395173765, 4.39671615001624,
                -2.53812272705723, 0.552910623085886]

        # 50Hz梳状工频陷波器（10阶）
        b_notch = self._generate_comb_filter_coefficients(0.897874972040025,
                                                           -0.897874972040025, 10)
        a_notch = self._generate_comb_filter_coefficients(1,
                                                           -0.795749944080050, 10)

        filters = [IirFilter(b_hp, a_hp), IirFilter(b_notch, a_notch)]
        self._filters_by_sampling_rate[500] = filters

    def _init_filters_for_1000(self):
        """初始化1000Hz采样率的滤波器组"""
        # 18Hz高通滤波器（4阶）
        b_hp = [0.862550850547179, -3.45020340218872, 5.17530510328307,
                -3.45020340218872, 0.862550850547179]
        a_hp = [1, -3.70453845798319, 5.15648369715095,
                -3.19579748376260, 0.743993969858123]

        # 50Hz梳状工频陷波器（20阶）
        b_notch = self._generate_comb_filter_coefficients(0.897874972040025,
                                                           -0.897874972040025, 20)
        a_notch = self._generate_comb_filter_coefficients(1,
                                                           -0.795749944080050, 20)

        filters = [IirFilter(b_hp, a_hp), IirFilter(b_notch, a_notch)]
        self._filters_by_sampling_rate[1000] = filters

    def _init_filters_for_2000(self):
        """初始化2000Hz采样率的滤波器组"""
        # 18Hz高通滤波器（6阶）
        b_hp = [0.896495417452848, -5.37897250471709, 13.4474312617927,
                -17.9299083490570, 13.4474312617927, -5.37897250471709,
                0.896495417452848]
        a_hp = [1, -5.78151843076305, 13.9313314341308,
                -17.9085161935195, 12.9528179055271, -4.99781871952794,
                0.803704033513957]

        # 50Hz梳状工频陷波器（40阶）
        b_notch = self._generate_comb_filter_coefficients(0.897874972040025,
                                                           -0.897874972040025, 40)
        a_notch = self._generate_comb_filter_coefficients(1,
                                                           -0.795749944080050, 40)

        filters = [IirFilter(b_hp, a_hp), IirFilter(b_notch, a_notch)]

        self._filters_by_sampling_rate[2000] = filters

    def _init_filters_for_4000(self):
        """初始化4000Hz采样率的滤波器组"""
        # 18Hz高通滤波器（6阶）
        b_hp = [0.946840974221639, -5.68104584532983, 14.2026146133246,
                -18.9368194844328, 14.2026146133246, -5.68104584532983,
                0.946840974221639]
        a_hp = [1, -5.89075707635233, 14.4597364082858,
                -18.9311699792352, 13.9426669365837, -5.47698411926291,
                0.896507830464982]

        # 50Hz梳状工频陷波器（80阶）
        b_notch = self._generate_comb_filter_coefficients(0.897874972040025,
                                                           -0.897874972040025, 80)
        a_notch = self._generate_comb_filter_coefficients(1,
                                                           -0.795749944080050, 80)

        filters = [IirFilter(b_hp, a_hp), IirFilter(b_notch, a_notch)]
        self._filters_by_sampling_rate[4000] = filters

    def process_sample(self, sampling_rate, input_sample):
        """
        处理单个肌电信号样本

        Args:
            sampling_rate: 采样率 (Hz)
            input_sample: 输入样本值

        Returns:
            滤波后的样本值
        """
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if not filters:
            return input_sample

        output = input_sample

        for filter_obj in filters:
            output = filter_obj.process_sample(output)

        return output

    def process_samples(self, sampling_rate, input_samples):
        """
        批量处理肌电信号样本

        Args:
            sampling_rate: 采样率 (Hz)
            input_samples: 输入样本列表或数组

        Returns:
            滤波后的样本列表
        """
        return [self.process_sample(sampling_rate, x) for x in input_samples]

    def reset_filters(self, sampling_rate):
        """
        重置指定采样率的所有滤波器状态

        Args:
            sampling_rate: 采样率 (Hz)
        """
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if filters:
            for filter_obj in filters:
                filter_obj.reset()

    def reset_all_filters(self):
        """重置所有采样率的所有滤波器状态"""
        for filters in self._filters_by_sampling_rate.values():
            for filter_obj in filters:
                filter_obj.reset()

    def supports_sampling_rate(self, sampling_rate):
        """
        检查是否支持指定的采样率

        Args:
            sampling_rate: 采样率 (Hz)

        Returns:
            是否支持该采样率
        """
        return sampling_rate in self._filters_by_sampling_rate

    def get_supported_sampling_rates(self):
        """
        获取支持的采样率列表

        Returns:
            支持的所有采样率列表
        """
        return list(self._filters_by_sampling_rate.keys())


# 使用示例
if __name__ == "__main__":
    # 创建滤波器实例
    emg_filter = EmgFilter()

    # 检查支持的采样率
    print("支持的采样率:", emg_filter.get_supported_sampling_rates())

    # 模拟处理几个样本（500Hz采样率）
    test_samples = [1.0, 0.5, -0.2, 0.1, -0.05] * 10
    filtered = emg_filter.process_samples(500, test_samples)
    print("滤波结果 (前10个):", filtered[:10])

    # 重置滤波器
    emg_filter.reset_filters(500)
    print("滤波器状态已重置")