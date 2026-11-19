# reroute_guard.py — 再ルートのガード (ステップ2)
# 目的: 1回のGNSS測定だけで再ルートしないこと。確認・間隔・回数上限・
# 新旧比較・予告なし変更をなくす。初学者向けに、標準ライブラリだけで
# 書いてある。ハードウェアには触らないので、このファイルだけ単体で
# テストできる (python3 test_step2.py)。
# 使い方: main.py と同じフォルダに置き、
#   from reroute_guard import RerouteGuard, is_same_route
# と読み込む。main_patched.py がその使い方になっている。
# 閾値の根拠はレビュー文書 (workspace/your_files/点字ピン方向定位デバイス_
# コードレビュー.md §8) による。20m・30秒・3回はいずれも提案値で、
# 現場テスト (ステップ7) で確かめる前の値である。

# --- 閾値 (提案値。現場テストで確かめること) ---
REROUTE_THRESHOLD_M = 20
REROUTE_CONFIRM_COUNT = 3       # 何回連続で超えたら再ルートするか
REROUTE_MIN_INTERVAL_SEC = 30.0  # 再ルートと再ルートのあいだの最短間隔
REROUTE_MAX_FAILURES = 3        # 連続で失敗したら再ルートを止める
SAME_ROUTE_BEARING_DEG = 22.5   # 最初の20mの方位差がこれ以内なら同じ経路
SAME_ROUTE_DIST_RATIO = 0.10    # 総距離の差がこれ以内なら同じ経路


def angle_diff(a, b):
    """2つの方位の差を -180〜180° で返す。"""
    return (a - b + 180) % 360 - 180


def is_same_route(old, new):
    """新しい経路が古い経路と「同じ」とみなせるか。
    old / new は fetch_route の戻り値 (coords / distances / bearings)。
    最初の方位の差が22.5°以内で、総距離の差が10%以内なら同じとする。
    どちらかでも欠けていたら「同じ」とは言わない (安全側に倒す)。"""
    try:
        old_b = old["bearings"][0]
        new_b = new["bearings"][0]
        old_total = old["distances"][-1]
        new_total = new["distances"][-1]
    except (KeyError, IndexError, TypeError):
        return False
    if old_total <= 0:
        return False
    if abs(angle_diff(old_b, new_b)) > SAME_ROUTE_BEARING_DEG:
        return False
    if abs(new_total - old_total) / old_total > SAME_ROUTE_DIST_RATIO:
        return False
    return True


class RerouteGuard:
    """再ルートしてよいかを決めるだけの小さな状態機械。
    now には time.monotonic() を渡す (時計が戻っても間隔が狂わない)。"""

    def __init__(self):
        self._count = 0          # 連続で閾値を超えた回数
        self._last_attempt = None  # 前回再ルートを試した時刻
        self._failures = 0       # 連続失敗回数
        self.stopped = False     # 連続失敗で再ルートを止めたか

    def should_reroute(self, distance_m, now):
        """いま再ルートしてよいか。True のときだけ呼び出し元が
        fetch_route を呼ぶ。False なら呼び出し元は案内を続けるか、
        緊急表示のまま次の測定を待つ。"""
        if self.stopped:
            return False
        if distance_m < REROUTE_THRESHOLD_M:
            self._count = 0
            return False
        self._count += 1
        if self._count < REROUTE_CONFIRM_COUNT:
            return False
        if (self._last_attempt is not None
                and now - self._last_attempt < REROUTE_MIN_INTERVAL_SEC):
            return False
        self._count = 0
        self._last_attempt = now
        return True

    def note_success(self):
        """再ルート (または「同じ経路なので上書きしない」判断) が
        済んだら呼ぶ。連続失敗回数を 0 に戻す。"""
        self._failures = 0

    def note_failure(self):
        """Valhalla の呼び出しなどが失敗したら呼ぶ。
        3回連続で失敗したら止まったことを True で返す。"""
        self._failures += 1
        if self._failures >= REROUTE_MAX_FAILURES:
            self.stopped = True
            return True
        return False
