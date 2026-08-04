
from typing import Dict, List, Optional
from iir_filter import IirFilter
from median_filter import MedianFilter

class EcgFilter:
    """
    心电信号(ECG)滤波器
    提供针对不同采样率的心电信号滤波处理
    """

    def __init__(self):
        self._filters_by_sampling_rate: Dict[int, List[IirFilter]] = {}
        self._smooth_rate: Dict[int, MedianFilter] = {}

        self._initialize_filters_for_125hz()
        self._initialize_filters_for_250hz()
        self._initialize_filters_for_500hz()
        self._initialize_filters_for_1000hz()
        self._initialize_filters_for_2000hz()

    # ----------------------------------------------------------------------
    # 初始化各采样率的滤波器组
    # ----------------------------------------------------------------------
    def _initialize_filters_for_125hz(self):
        fs = 125
        filters = []

        # 50Hz工频陷波器（单极点）
        notch_num = [0.880317952033614, 1.42438436729709, 0.880317952033614]
        notch_den = [1, 1.42438436729709, 0.760635904067228]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[fs] = filters
        self._smooth_rate[fs] = MedianFilter(int(fs * 1.5))

    def _initialize_filters_for_250hz(self):
        fs = 250
        filters = []

        # 40Hz低通滤波器（6阶巴特沃斯）
        low_pass_num = [0.00353827357783166, 0.0212296414669899, 0.0530741036674748,
                        0.0707654715566331, 0.0530741036674748, 0.0212296414669899,
                        0.00353827357783166]
        low_pass_den = [1, -2.14075592419305, 2.50058256607543, -1.68559960742854,
                        0.697562920979370, -0.161779875180172, 0.0164394287281892]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.936813880017972, -0.578982818983773, 0.936813880017972]
        notch_den = [1, -0.578982818983773, 0.873627760035944]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[fs] = filters
        self._smooth_rate[fs] = MedianFilter(int(fs * 1.5))

    def _initialize_filters_for_500hz(self):
        fs = 500
        filters = []

        # 40Hz低通滤波器（6阶巴特沃斯）
        low_pass_num = [0.000107068897421403, 0.000642413384528420, 0.00160603346132105,
                        0.00214137794842807, 0.00160603346132105, 0.000642413384528420,
                        0.000107068897421403]
        low_pass_den = [1, -4.06164399921344, 7.09950381875071, -6.78501602539750,
                        3.72301942889160, -1.10867085534353, 0.139660041747139]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.967437388703836, -1.56534657691025, 0.967437388703836]
        notch_den = [1, -1.56534657691025, 0.934874777407671]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[fs] = filters
        self._smooth_rate[fs] = MedianFilter(int(fs * 1.5))

    def _initialize_filters_for_1000hz(self):
        fs = 1000
        filters = []

        # 40Hz低通滤波器（6阶巴特沃斯）
        low_pass_num = [2.49722252689037e-06, 1.49833351613422e-05, 3.74583379033556e-05,
                        4.99444505378075e-05, 3.74583379033556e-05, 1.49833351613422e-05,
                        2.49722252689037e-06]
        low_pass_den = [1, -5.02943835142161, 10.6070421837797, -11.9993158162167,
                        7.67547454820020, -2.63105512847395, 0.377452386374089]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.983457100401067, -1.87064656766634, 0.983457100401067]
        notch_den = [1, -1.87064656766634, 0.966914200802133]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[fs] = filters
        self._smooth_rate[fs] = MedianFilter(int(fs * 1.5))

    def _initialize_filters_for_2000hz(self):
        fs = 2000
        filters = []

        # 40Hz低通滤波器（6阶巴特沃斯）
        low_pass_num = [4.86398750078084e-08, 2.91839250046850e-07, 7.29598125117126e-07,
                        9.72797500156168e-07, 7.29598125117126e-07, 2.91839250046850e-07,
                        4.86398750078084e-08]
        low_pass_den = [1, -5.51453512116617, 12.6891130565151, -15.5936352107041,
                        10.7932966704854, -3.98935940423088, 0.615123122052628]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.991660562744315, -1.95890315130115, 0.991660562744315]
        notch_den = [1, -1.95890315130115, 0.983321125488630]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[fs] = filters
        self._smooth_rate[fs] = MedianFilter(int(fs * 1.5))

    # ----------------------------------------------------------------------
    # 公共接口
    # ----------------------------------------------------------------------
    def process_sample(self, sampling_rate: int, input_sample: float) -> float:
        """
        处理单个心电信号样本

        参数:
            sampling_rate: 采样率 (Hz)
            input_sample: 输入样本值

        返回:
            滤波后的样本值
        """
        output = input_sample

        # 去基线：减去中值滤波器的输出
        smooth_filter = self._smooth_rate.get(sampling_rate)
        if smooth_filter:
            output = output - smooth_filter.add_sample(output)

        # 去高频和工频：依次通过 IIR 滤波器
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if filters:
            for f in filters:
                output = f.process_sample(output)

        return output

    def process_samples(self, sampling_rate: int, input_samples: List[float]) -> List[float]:
        """
        批量处理心电信号样本

        参数:
            sampling_rate: 采样率 (Hz)
            input_samples: 输入样本列表

        返回:
            滤波后的样本列表
        """
        if input_samples is None:
            raise ValueError("input_samples cannot be None")
        return [self.process_sample(sampling_rate, x) for x in input_samples]

    def reset_filters(self, sampling_rate: int) -> None:
        """
        重置指定采样率的所有 IIR 滤波器状态
        """
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if filters:
            for f in filters:
                f.reset()
        # 中值滤波器的状态也重置？原 C# 代码没有重置中值滤波器，但可以加一个可选重置
        smooth_filter = self._smooth_rate.get(sampling_rate)
        if smooth_filter:
            smooth_filter.reset()

    def reset_all_filters(self) -> None:
        """重置所有采样率的所有 IIR 滤波器状态"""
        for filters in self._filters_by_sampling_rate.values():
            for f in filters:
                f.reset()
        for smooth_filter in self._smooth_rate.values():
            smooth_filter.reset()

    def supports_sampling_rate(self, sampling_rate: int) -> bool:
        """检查是否支持指定的采样率"""
        return sampling_rate in self._filters_by_sampling_rate

    def get_supported_sampling_rates(self) -> List[int]:
        """获取支持的采样率列表"""
        return list(self._filters_by_sampling_rate.keys())


# 使用示例
if __name__ == "__main__":
    ecg_filter = EcgFilter()
    print("支持的采样率:", ecg_filter.get_supported_sampling_rates())

    # 模拟 250Hz 采样率下的几个样本
    test_samples = [0.5, 0.6, 0.55, 0.7, 0.65]
    filtered = ecg_filter.process_samples(250, test_samples)
    print("滤波结果:", filtered)