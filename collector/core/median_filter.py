from collections import deque
import bisect

class MedianFilter:
    """
    一维中值滤波器，模拟 MATLAB 的 medfilt1 函数。
    支持逐点输入、逐点输出，窗口大小可动态调整。
    内部维护队列（保持时间顺序）和有序列表（快速计算中位数）。
    窗口未满时基于已有数据计算中位数，偶数窗口取两中间值的平均。
    """

    def __init__(self, window_size):
        """
        初始化中值滤波器。

        参数:
            window_size (int): 窗口大小（建议奇数，偶数时取平均）
        """
        if window_size <= 0:
            raise ValueError("窗口大小必须为正整数。")
        self._window_size = window_size
        self._queue = deque()          # 按时间顺序存储窗口内的值
        self._sorted = []               # 当前窗口内值的升序列表（通过bisect维护）

    @property
    def window_size(self):
        return self._window_size

    @window_size.setter
    def window_size(self, value):
        """设置窗口大小，若新窗口小于当前数据量，自动丢弃最旧数据。"""
        if value <= 0:
            raise ValueError("窗口大小必须为正整数。")
        if value < self._window_size:
            # 丢弃最早的数据，直到队列长度 ≤ 新窗口大小
            while len(self._queue) > value:
                oldest = self._queue.popleft()
                # 在有序列表中移除最先出现的该值（只移除一个）
                idx = bisect.bisect_left(self._sorted, oldest)
                if idx < len(self._sorted) and self._sorted[idx] == oldest:
                    self._sorted.pop(idx)
        # 新窗口更大无需立即丢弃
        self._window_size = value

    @property
    def count(self):
        """当前窗口内有效样本数。"""
        return len(self._queue)

    @property
    def is_window_full(self):
        """窗口是否已填满（count >= window_size）。"""
        return len(self._queue) >= self._window_size

    def reset(self):
        """重置滤波器，清空所有历史数据。"""
        self._queue.clear()
        self._sorted.clear()

    def add_sample(self, sample):
        """
        输入一个新采样点，返回当前窗口的中位值。

        参数:
            sample (float): 新采样值

        返回:
            float: 滑动中位值；若窗口为空则返回 float('nan')
        """
        # 1. 将新值加入队列和有序列表
        self._queue.append(sample)
        bisect.insort(self._sorted, sample)

        # 2. 若超出窗口大小，移除最旧的值
        if len(self._queue) > self._window_size:
            oldest = self._queue.popleft()
            # 在有序列表中移除最先出现的该值
            idx = bisect.bisect_left(self._sorted, oldest)
            if idx < len(self._sorted) and self._sorted[idx] == oldest:
                self._sorted.pop(idx)

        # 3. 计算当前中位数
        if not self._sorted:
            return float('nan')
        mid = len(self._sorted) // 2
        if len(self._sorted) % 2 == 1:
            # 奇数个元素：正中间
            return self._sorted[mid]
        else:
            # 偶数个元素：中间两数平均
            return (self._sorted[mid - 1] + self._sorted[mid]) / 2.0

    def process(self, data):
        """
        一次性处理整个数组（批处理模式），返回滤波后数组。

        参数:
            data (list or array-like): 输入信号数组

        返回:
            list: 滤波结果数组，长度与输入相同
        """
        result = []
        for x in data:
            result.append(self.add_sample(x))
        return result

    def __repr__(self):
        return f"MedianFilter(window_size={self._window_size})"