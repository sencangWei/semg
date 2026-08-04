class IirFilter:
    """
    通用 IIR 滤波器，支持自动检测梳状结构并优化计算

    参数:
        numerator_coefficients (list of float): 分子系数 b (b0, b1, b2, ...)
        denominator_coefficients (list of float): 分母系数 a (a0, a1, a2, ...)，a0 不能为零

    方法:
        process_sample(x) : 处理单个样本，根据系数结构自动选择算法
        process_samples(x_list) : 批量处理样本列表
        reset() : 重置滤波器内部状态
    """

    def __init__(self, numerator_coefficients, denominator_coefficients):
        if not denominator_coefficients or denominator_coefficients[0] == 0:
            raise ValueError("分母系数不能为空且 a0 不能为零")

        self.b = [float(c) for c in numerator_coefficients]   # 分子系数
        self.a = [float(c) for c in denominator_coefficients] # 分母系数

        # 检测是否为梳状滤波器（稀疏系数）
        self._is_comb = self._detect_comb()
        if self._is_comb:
            # 提取梳状参数
            self._init_comb()
        else:
            # 通用滤波器：初始化历史缓冲区
            self._input_history = [0.0] * len(self.b)
            self._output_history = [0.0] * (len(self.a) - 1)

    def _detect_comb(self, eps=1e-12):
        """
        检测系数是否符合梳状结构：
        - 分子系数中，除 b[0] 和最后一个非零系数外，其余皆为零
        - 分母系数中，除 a[0] 和最后一个非零系数外，其余皆为零
        - a[0] 应接近于 1
        """
        # 检查 a0 是否接近 1
        if abs(self.a[0] - 1.0) > eps:
            return False

        # 找到分子最后一个非零系数的索引
        b_last = -1
        for i in range(len(self.b)-1, -1, -1):
            if abs(self.b[i]) > eps:
                b_last = i
                break

        # 找到分母最后一个非零系数的索引
        a_last = -1
        for i in range(len(self.a)-1, -1, -1):
            if abs(self.a[i]) > eps:
                a_last = i
                break

        # 如果没有非零项（不应发生，但以防万一）
        if b_last == -1 and a_last == -1:
            return False

        # 取最大索引作为延迟阶数 N
        self._comb_delay = max(b_last, a_last)
        N = self._comb_delay

        # 检查中间系数是否为零
        # 分子：除索引 0 和 N 外应全零
        for i, val in enumerate(self.b):
            if i == 0 or i == N:
                continue
            if abs(val) > eps:
                return False

        # 分母：除索引 0 和 N 外应全零
        for i, val in enumerate(self.a):
            if i == 0 or i == N:
                continue
            if abs(val) > eps:
                return False

        return True

    def _init_comb(self):
        """初始化梳状滤波器的参数和缓冲区"""
        N = self._comb_delay
        # 提取系数
        self._comb_b0 = self.b[0] if len(self.b) > 0 else 0.0
        self._comb_bN = self.b[N] if N < len(self.b) else 0.0
        self._comb_aN = self.a[N] if N < len(self.a) else 0.0

        # 循环缓冲区，存储最近 N 个输入和输出
        self._comb_x_buf = [0.0] * N
        self._comb_y_buf = [0.0] * N
        self._comb_idx = 0  # 当前要覆盖的位置（最旧的数据）

    def reset(self):
        """重置滤波器状态"""
        if self._is_comb:
            self._comb_x_buf = [0.0] * self._comb_delay
            self._comb_y_buf = [0.0] * self._comb_delay
            self._comb_idx = 0
        else:
            for i in range(len(self._input_history)):
                self._input_history[i] = 0.0
            for i in range(len(self._output_history)):
                self._output_history[i] = 0.0

    def process_sample_comb(self, x):
        """
        梳状滤波器专用处理函数（高效实现）
        差分方程：y[n] = b0*x[n] + bN*x[n-N] - aN*y[n-N]
        """
        N = self._comb_delay
        if N == 0:   # 延迟为0时直接计算（退化情况）
            return (self._comb_b0 * x) / self.a[0]

        # 从缓冲区读取最旧的延迟样本
        x_old = self._comb_x_buf[self._comb_idx]
        y_old = self._comb_y_buf[self._comb_idx]

        # 计算当前输出（注意：a0=1，无需除法）
        y = self._comb_b0 * x + self._comb_bN * x_old - self._comb_aN * y_old

        # 更新缓冲区（覆盖最旧位置）
        self._comb_x_buf[self._comb_idx] = x
        self._comb_y_buf[self._comb_idx] = y

        # 移动指针到下一个最旧位置
        self._comb_idx = (self._comb_idx + 1) % N

        return y

    def process_sample(self, x):
        """
        处理单个输入样本，自动选择算法
        """
        if self._is_comb:
            return self.process_sample_comb(x)
        else:
            # 通用直接I型算法
            # 更新输入历史：向后移动
            for i in range(len(self._input_history) - 1, 0, -1):
                self._input_history[i] = self._input_history[i - 1]
            self._input_history[0] = x

            # 分子部分
            y = 0.0
            for i in range(len(self.b)):
                y += self.b[i] * self._input_history[i]

            # 减去分母反馈（忽略 a0）
            for i in range(len(self._output_history)):
                y -= self.a[i + 1] * self._output_history[i]

            # 除以 a0
            y /= self.a[0]

            # 更新输出历史
            for i in range(len(self._output_history) - 1, 0, -1):
                self._output_history[i] = self._output_history[i - 1]
            if len(self._output_history) > 0:
                self._output_history[0] = y

            return y

    def process_samples(self, samples):
        """
        批量处理多个样本
        """
        if not isinstance(samples, (list, tuple)):
            samples = list(samples)

        outputs = []
        for x in samples:
            outputs.append(self.process_sample(x))
        return outputs


# 使用示例
if __name__ == "__main__":
    # 示例1：梳状滤波器（500Hz 50Hz陷波器）
    b_notch = [0.897874972040025] + [0.0]*9 + [-0.897874972040025]  # 长度 11 (N=10)
    a_notch = [1.0] + [0.0]*9 + [-0.795749944080050]                 # 长度 11 (N=10)

    comb_filter = IirFilter(b_notch, a_notch)
    print("是否为梳状滤波器:", comb_filter._is_comb)  # 应输出 True

    # 测试梳状脉冲响应
    test_input = [1.0] + [0.0]*20
    output = comb_filter.process_samples(test_input)
    print("梳状滤波器脉冲响应:", [f"{v:.5f}" for v in output])

    # 示例2：通用低通滤波器
    b_lp = [0.0675, 0.1349, 0.0675]
    a_lp = [1.0, -1.1430, 0.4128]

    generic_filter = IirFilter(b_lp, a_lp)
    print("是否为梳状滤波器:", generic_filter._is_comb)  # 应输出 False

    # 测试通用脉冲响应
    output2 = generic_filter.process_samples([1.0, 0, 0, 0, 0])
    print("通用滤波器脉冲响应:", [f"{v:.4f}" for v in output2])