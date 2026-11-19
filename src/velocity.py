"""歩く速度（分速）を、直近の窓の平均で返す。

GNSS の fix には受信機が出す対地速度 speed_mps が入っている（GNSS.py が
NAV-PVT の gSpeed を保持している）。1回ごとの値は歩行速度では揺れるので、
直近 window_sec 秒ぶんを単純平均する。

「裏でずっと測る」実体は GNSS 側の読み取りスレッドで、このクラスは
main のループ（1秒周期）が毎回 add() して平均を保つだけの軽いもの。
"""
import time
from collections import deque


class WalkSpeed:
    def __init__(self, window_sec=30.0):
        self.window_sec = float(window_sec)
        self._samples = deque()  # (単調時刻, 速度 m/s)

    def add(self, speed_mps, mono=None):
        mono = time.monotonic() if mono is None else mono
        self._samples.append((mono, float(speed_mps)))
        while self._samples and mono - self._samples[0][0] > self.window_sec:
            self._samples.popleft()

    def span_sec(self):
        """溜まっているサンプルの時間幅 [s]"""
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    def ready(self, min_span_sec=10.0):
        """閾値の計算に使えるだけ溜まったか"""
        return self.span_sec() >= min_span_sec

    def speed_mps(self):
        """平均の歩行速度 [m/s]。サンプルが無ければ None"""
        if not self._samples:
            return None
        return sum(v for _, v in self._samples) / len(self._samples)

    def per_min(self):
        """分速 [m/分]。サンプルが無ければ None"""
        v = self.speed_mps()
        return None if v is None else v * 60.0
