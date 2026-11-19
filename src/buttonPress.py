"""タクトスイッチの押下と、ソレノイドの点滅（コーナー警告）の対応を記録する。

記録先は corners.txt（main が渡す）。書く行は4種類。

- PRESS / RELEASE：ボタンを押した・離した瞬間（壁時計ミリ秒＋単調時計）。
  PRESS 行には、点滅中なら点滅開始からの経過（blink_delta）も併記する
- BLINK：点滅1回分の区間「開始 — 終了」と、その長さ（点滅時間）
- DIFF：点滅と、それに重なった押下を対応付けた差分
    認識遅延 = 押した時刻 − 点滅開始時刻
    終了差   = 離した時刻 − 点滅終了時刻
    長さの差 = 押していた時間 − 点滅していた時間
  点滅中に押下が無ければ、その旨の行（押下なし）を書く

点滅の開始・終了は main が note_blink_start() / note_blink_end() で知らせる。
main は1秒周期のループで点滅の切り替わりを検出するので、点滅側の時刻には
最大で約1秒の量子化が入る。ボタン側は押した瞬間に記録される。

時刻は2種類使う：人が読む壁時計（ミリ秒まで）と、差の計算用の単調時計
（time.monotonic）。壁時計は途中で補正されてずれることがあるので、
差を取るときは単調時計どうしで計算する。
配線は GPIO と GND の間にスイッチ、内部プルアップ有効（gpiozero の Button）。
チャタリング対策に bounce_time=0.05 を入れている。
"""
import datetime
import os
import threading
import time

from gpiozero import Button


def _wall_ms():
    now = datetime.datetime.now()
    return now.strftime("%Y-%m-%d %H:%M:%S.") + f"{now.microsecond // 1000:03d}"


class ButtonLogger:
    """pin 番号のタクトスイッチを監視し、押下と点滅の対応を log_path に書く。"""

    def __init__(self, pin, log_path):
        self._log_path = log_path
        self._lock = threading.Lock()
        self._press = None    # 押下中：{"mono", "wall"}
        self._pending = []    # 解放済みで、点滅の終了待ちの押下
        self._blink = None    # 直近の点滅：{"start_mono", "start_wall", "end_mono", "end_wall"}
        self._button = Button(pin, pull_up=True, bounce_time=0.05)
        self._button.when_pressed = self._on_press
        self._button.when_released = self._on_release
        self._write(f"--- button logger start (GPIO{pin}) ---")

    # ---------- 記録 ----------
    def _write(self, line):
        with self._lock:
            os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
            with open(self._log_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")

    # ---------- main からの点滅通知 ----------
    def note_blink_start(self, mono=None):
        mono = time.monotonic() if mono is None else mono
        with self._lock:
            if self._blink is not None and self._blink["end_mono"] is None:
                # 終了を取りこぼした点滅は、新しい開始時刻で閉じておく
                self._close_blink(mono)
            self._blink = {"start_mono": mono, "start_wall": _wall_ms(),
                           "end_mono": None, "end_wall": None}

    def note_blink_end(self, mono=None):
        mono = time.monotonic() if mono is None else mono
        with self._lock:
            if self._blink is not None and self._blink["end_mono"] is None:
                self._close_blink(mono)

    def _close_blink(self, end_mono):
        """点滅区間を確定し、BLINK 行と、対応する DIFF 行（または押下なし）を書く。

        self._lock を持った状態で呼ぶこと（_write が同じロックを取るため、
        ここでは _write を使わず直接書く）。
        """
        b = self._blink
        b["end_mono"] = end_mono
        b["end_wall"] = _wall_ms()
        blink_dur = end_mono - b["start_mono"]
        lines = [f"BLINK [{b['start_wall']}] — [{b['end_wall']}] "
                 f"点滅時間={blink_dur:.2f}s"]

        paired = 0
        rest = []
        for p in self._pending:
            if self._overlaps(p, b):
                lines.append(self._diff_line(p, b))
                paired += 1
            else:
                rest.append(p)
        self._pending = rest

        if self._press is not None and self._press["mono"] <= end_mono:
            # 点滅中に押している最中（開始が点滅前でもよい）。DIFF は離した時点で書く
            paired += 1
        if paired == 0:
            lines.append(f"DIFF なし（点滅 [{b['start_wall']}] の間に押下なし）")

        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        with open(self._log_path, "a", encoding="utf-8") as f:
            for line in lines:
                f.write(line + "\n")

    @staticmethod
    def _overlaps(press, blink):
        """押下区間と点滅区間が重なっているか"""
        return press["mono"] <= blink["end_mono"] \
            and press["release_mono"] >= blink["start_mono"]

    @staticmethod
    def _diff_line(press, blink):
        delay = press["mono"] - blink["start_mono"]              # 認識遅延
        end_diff = press["release_mono"] - blink["end_mono"]     # 終了差
        press_dur = press["release_mono"] - press["mono"]        # 押していた時間
        blink_dur = blink["end_mono"] - blink["start_mono"]      # 点滅していた時間
        return (f"DIFF PRESS [{press['wall']}] — [{press['release_wall']}] "
                f"押下時間={press_dur:.2f}s | 認識遅延={delay:.2f}s "
                f"終了差={end_diff:.2f}s 長さの差={press_dur - blink_dur:.2f}s")

    # ---------- ボタンのコールバック ----------
    def _on_press(self):
        mono = time.monotonic()
        wall = _wall_ms()
        with self._lock:
            self._press = {"mono": mono, "wall": wall}
            delta = None
            if self._blink is not None and self._blink["end_mono"] is None:
                delta = mono - self._blink["start_mono"]
        tail = f" blink_delta={delta:.2f}s" if delta is not None else " blink_delta=-"
        self._write(f"[{wall}] PRESS mono={mono:.3f}{tail}")

    def _on_release(self):
        mono = time.monotonic()
        wall = _wall_ms()
        diff_line = None
        with self._lock:
            if self._press is None:
                return
            press = dict(self._press)
            self._press = None
            press["release_mono"] = mono
            press["release_wall"] = wall
            dur = mono - press["mono"]
            blink = self._blink
            if blink is not None and blink["end_mono"] is not None \
                    and self._overlaps(press, blink):
                # 点滅は終わっているので、その場で DIFF を書く
                diff_line = self._diff_line(press, blink)
            elif blink is not None and blink["end_mono"] is None \
                    and press["release_mono"] >= blink["start_mono"]:
                # 点滅がまだ続いている。DIFF は点滅が終わった時点で書く
                self._pending.append(press)
        self._write(f"[{wall}] RELEASE mono={mono:.3f} press_duration={dur:.2f}s")
        if diff_line is not None:
            self._write(diff_line)
