from typing import Dict, List, Optional
from iir_filter import IirFilter

class EegFilter:
    """
    脑电信号(EEG)滤波器
    提供针对不同采样率的脑电信号滤波处理
    """

    def __init__(self):
        """初始化脑电信号滤波器，自动配置 125、250、500、1000、2000 Hz 采样率的滤波器"""
        self._filters_by_sampling_rate: Dict[int, List[IirFilter]] = {}

        self._initialize_filters_for_125hz()
        self._initialize_filters_for_250hz()
        self._initialize_filters_for_500hz()
        self._initialize_filters_for_1000hz()
        self._initialize_filters_for_2000hz()

    # ----------------------------------------------------------------------
    # 初始化各采样率的滤波器组
    # ----------------------------------------------------------------------
    def _initialize_filters_for_125hz(self):
        """初始化 125Hz 采样率的滤波器组"""
        filters = []

        # 0.5Hz 高通滤波器（4阶巴特沃斯）
        high_pass_num = [0.967694808889672, -3.87077923555869, 5.80616885333803,
                         -3.87077923555869, 0.967694808889672]
        high_pass_den = [1, -3.93432582079874, 5.80512542105514,
                         -3.80723245722885, 0.936433243152019]
        filters.append(IirFilter(high_pass_num, high_pass_den))

        # 50Hz 工频陷波器（单极点）
        notch_num = [0.917047349314333, 1.48381378048359, 0.917047349314333]
        notch_den = [1, 1.48381378048359, 0.834094698628666]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[125] = filters

    def _initialize_filters_for_250hz(self):
        """初始化 250Hz 采样率的滤波器组"""
        filters = []

        # 0.5Hz 高通滤波器（4阶巴特沃斯）
        high_pass_num = [0.983715174129757, -3.93486069651903, 5.90229104477854,
                         -3.93486069651903, 0.983715174129757]
        high_pass_den = [1, -3.96716259594885, 5.90202586149088,
                         -3.90255878482324, 0.967695543813137]
        filters.append(IirFilter(high_pass_num, high_pass_den))

        # 45Hz低通滤波器（6阶巴特沃斯）
        low_pass_num = [0.00624027613969259	,0.0374416568381555,	0.0936041420953888,	0.124805522793852	,0.0936041420953888,	0.0374416568381555,	0.00624027613969259]
        low_pass_den = [1,	-1.66368697588776,	1.81520557966088	,-1.09968415154832,	0.432169089539191,	-0.0937901346011633	,0.00916426577749563]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.936813880017972, -0.578982818983773, 0.936813880017972]
        notch_den = [1, -0.578982818983773, 0.873627760035944]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[250] = filters

    def _initialize_filters_for_500hz(self):
        """初始化 500Hz 采样率的滤波器组"""
        filters = []

        # 0.5Hz 高通滤波器（5阶巴特沃斯）
        high_pass_num = [0.989885075391028, -4.94942537695514, 9.89885075391028,
                         -9.89885075391028, 4.94942537695514, -0.989885075391028]
        high_pass_den = [1, -4.97966719499007, 9.91887533813754,
                         -9.87862154877962, 4.91928586812375, -0.979872462481901]
        filters.append(IirFilter(high_pass_num, high_pass_den))

        # 45Hz 低通滤波器（6阶巴特沃斯）
        low_pass_num = [0.000197907656847044	,0.00118744594108226	,0.00296861485270566,	0.00395815313694087,	0.00296861485270566,	0.00118744594108226	,0.000197907656847044]
        low_pass_den = [1,	-3.82038044485157,	6.35380263251389,	-5.81892155563278	,3.07582737735020	,-0.885977249058745	,0.108315329717220]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.967437388703836, -1.56534657691025, 0.967437388703836]
        notch_den = [1, -1.56534657691025, 0.934874777407671]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[500] = filters

    def _initialize_filters_for_1000hz(self):
        """初始化 1000Hz 采样率的滤波器组"""
        filters = []

        # 0.5Hz 高通滤波器（5阶巴特沃斯）
        high_pass_num = [0.994929691353824, -4.97464845676912, 9.94929691353824,
                         -9.94929691353824, 4.97464845676912, -0.994929691353824]
        high_pass_den = [1, -4.98983359383530, 9.95938603408544,
                         -9.93915637708832, 4.95948902757590, -0.989885090737415]
        filters.append(IirFilter(high_pass_num, high_pass_den))

        # 45Hz 低通滤波器（6阶巴特沃斯）
        low_pass_num = [4.80161878585565e-06,	2.88097127151339e-05,	7.20242817878347e-05	,9.60323757171129e-05,	7.20242817878347e-05,	2.88097127151339e-05,	4.80161878585565e-06]
        low_pass_den = [1,	-4.90826232220073,	10.1214192938314	,-11.2134236537737,	7.03426952731397	,-2.36754043931159	,0.333844897742978]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.983457100401067, -1.87064656766634, 0.983457100401067]
        notch_den = [1, -1.87064656766634, 0.966914200802133]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[1000] = filters

    def _initialize_filters_for_2000hz(self):
        """初始化 2000Hz 采样率的滤波器组"""
        filters = []

        # 0.5Hz 高通滤波器（4阶巴特沃斯）
        high_pass_num = [0.997949760065881, -3.99179904026353, 5.98769856039529,
                         -3.99179904026353, 0.997949760065881]
        high_pass_den = [1, -3.99589531159288, 5.98769435691454,
                         -3.98770276893113, 0.995903723615550]
        filters.append(IirFilter(high_pass_num, high_pass_den))

        # 45Hz 低通滤波器（6阶巴特沃斯）
        low_pass_num = [9.58526452824651e-08,	5.75115871694790e-07,	1.43778967923698e-06	,1.91705290564930e-06,	1.43778967923698e-06	,5.75115871694790e-07	,9.58526452824651e-08]
        low_pass_den = [1	,-5.45387055757060,	12.4164881877990,	-15.1024974088935,	10.3499935623799,	-3.78890907858704	,0.578801429441506]
        filters.append(IirFilter(low_pass_num, low_pass_den))

        # 50Hz工频陷波器（单极点）
        notch_num = [0.991660562744315, -1.95890315130115, 0.991660562744315]
        notch_den = [1, -1.95890315130115, 0.983321125488630]
        filters.append(IirFilter(notch_num, notch_den))

        self._filters_by_sampling_rate[2000] = filters

    # ----------------------------------------------------------------------
    # 公共接口
    # ----------------------------------------------------------------------
    def process_sample(self, sampling_rate: int, input_sample: float) -> float:
        """
        处理单个脑电信号样本

        参数:
            sampling_rate: 采样率 (Hz)
            input_sample: 输入样本值

        返回:
            滤波后的样本值
        """
        output = input_sample
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if filters:
            for f in filters:
                output = f.process_sample(output)
        return output

    def process_samples(self, sampling_rate: int, input_samples: List[float]) -> List[float]:
        """
        批量处理脑电信号样本

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
        重置指定采样率的所有滤波器状态

        参数:
            sampling_rate: 采样率 (Hz)
        """
        filters = self._filters_by_sampling_rate.get(sampling_rate)
        if filters:
            for f in filters:
                f.reset()

    def reset_all_filters(self) -> None:
        """重置所有采样率的所有滤波器状态"""
        for filters in self._filters_by_sampling_rate.values():
            for f in filters:
                f.reset()

    def supports_sampling_rate(self, sampling_rate: int) -> bool:
        """检查是否支持指定的采样率"""
        return sampling_rate in self._filters_by_sampling_rate

    def get_supported_sampling_rates(self) -> List[int]:
        """获取支持的采样率列表"""
        return list(self._filters_by_sampling_rate.keys())

    # ----------------------------------------------------------------------
    # 辅助方法
    # ----------------------------------------------------------------------
    @staticmethod
    def _generate_comb_filter_coefficients(start_value: float, end_value: float,
                                           filter_order: int) -> List[float]:
        """
        生成梳状滤波器系数：长度为 filter_order+1，第一个元素为 start_value，
        最后一个元素为 end_value，中间填充零。

        参数:
            start_value: 起始系数值
            end_value: 结束系数值
            filter_order: 滤波器阶数（决定了零的个数）

        返回:
            系数列表
        """
        coeffs = [start_value] + [0.0] * (filter_order - 1) + [end_value]
        return coeffs