import time
import threading
from gpiozero import LED, DigitalInputDevice



# SDA:pin2  SCL:pin3  TXD:pin14  RXD:pin15
GPIOS=[23,24,25,8,7,1,12,16,20]
# 使うgpio全部をインスタンス化

N_SOLENOIDS = 6      # ①〜⑥

BLINK_INTERVAL = {"blink_slow": 1.0,"blink": 0.5,"blink_fast": 0.3,}
WAVE_INTERVAL=0.5

# ---- 実際に挙上したかを調べるための入力 (配線に検出用の部品を足した場合だけ設定する) ----
# {ソレノイド番号(1〜6): 検出用のGPIO番号}   例: {1: 5, 2: 6}
# 検出用の部品の例: プランジャーの位置を見るリミットスイッチやホールセンサー (機械的な挙上を検出できる)
#                  電流センサー(INA219など)は、コイルに電流が流れたか(断線していないか)までは分かるが、動いたかは分からない
SENSE_GPIOS = {}
SENSE_ACTIVE_HIGH = True    # 検出したときに、入力がHIGHになる配線ならTrue (LOWになるならFalse)

_pins = [LED(gpio, initial_value=False) for gpio in GPIOS]
_sense = {i: DigitalInputDevice(g, pull_up=not SENSE_ACTIVE_HIGH) for i, g in SENSE_GPIOS.items()}
isActive=[]

_state_lock = threading.Lock()
_current = set()

def LowAll():
    global _current
    with _state_lock:
        for p in _pins[:N_SOLENOIDS]: p.off()
        _current = set()

def emergency():
    global _current
    with _state_lock:
        for p in _pins[:N_SOLENOIDS]: p.on()
        _current = set(range(1, N_SOLENOIDS + 1))

def status():
    """いまのソレノイドの状態を調べる。 (ソフトが出力しているはずの状態 と 実際の状態の比較)
      applied  : ソフトが最後にGPIOへ書き込んだ集合 (いま挙上するように出力しているはずの番号。点滅中はその瞬間の状態)
      output   : GPIOピンを読み戻した、実際の出力レベルがHIGHの番号
      sensed   : 検出用の入力が「挙上した」と答えている番号 (SENSE_GPIOS を設定していなければ None)
      output_mismatch : applied と output が食い違う番号 (ピンの競合などで、出力できていない)
      sensed_mismatch : 検出用の入力がある番号のうち、applied と sensed が食い違う番号 (挙上できていない)
    注意: sensed は、命令してから少し(数十ミリ秒〜)遅れて変わる。 命令の直後に比べると、食い違って見える。"""
    with _state_lock:
        applied = set(_current)
        output = {i for i in range(1, N_SOLENOIDS + 1) if _pins[i - 1].value}
    sensed = None
    sensed_mismatch = None
    if _sense:
        sensed = {i for i, d in _sense.items() if d.value}
        keys = set(_sense)
        sensed_mismatch = sorted((applied & keys) ^ sensed)
    return dict(applied=sorted(applied), output=sorted(output),
                sensed=None if sensed is None else sorted(sensed),
                output_mismatch=sorted(applied ^ output), sensed_mismatch=sensed_mismatch)

def _apply(target):
    global _current
    target = set(target)
    with _state_lock:
        for i in _current - target: _pins[i-1].off()
        for i in target - _current: _pins[i-1].on()
        _current = target


class Display:
    def __init__(self):
        self._lock = threading.Lock()
        self._set, self._mode = set(), "solid"
        self._waveiv = WAVE_INTERVAL
        self._paused = False
        threading.Thread(target=self._raise, daemon=True).start()
    def set(self, sole_set, mode,wave_iv=WAVE_INTERVAL):
        with self._lock:
            self._set, self._mode,self._waveiv = set(sole_set), mode, wave_iv
    def status(self):
        """status() に、命令した内容 (commanded, mode) を加えて返す"""
        with self._lock:
            commanded, mode = sorted(self._set), self._mode
        st = status()
        st.update(commanded=commanded, mode=mode)
        return st
    def selftest(self, wait=0.4):
        """ソレノイドを1つずつ挙上させて、出力と(検出用の入力があれば)挙上を確認する。
        戻り値: {番号: "OK" / "出力NG" / "挙上NG" / "未確認(検出用の入力なし)"}
        配線が外れていないかの確認に使う。 検出用の入力がないと、「未確認」までしか分からない"""
        self._paused = True
        time.sleep(0.1)                      # 表示用スレッドが止まるのを待つ
        result = {}
        try:
            for i in range(1, N_SOLENOIDS + 1):
                _apply({i})
                time.sleep(wait)
                st = status()
                if i not in st["output"]:
                    result[i] = "出力NG"
                elif i in _sense:
                    result[i] = "OK" if i in st["sensed"] else "挙上NG"
                else:
                    result[i] = "未確認(検出用の入力なし)"
                _apply(set())
                time.sleep(0.05)
        finally:
            _apply(set())
            self._paused = False
        return result
    def _raise(self):
        while True:
            if self._paused:
                time.sleep(0.01)
                continue
            with self._lock:
                s, m, w_iv = self._set, self._mode,self._waveiv
            if m == "solid":
                _apply(s)
                time.sleep(0.02)
            elif m == "wave":
                # 到着の波: 左の列を0.5秒、右の列を0.5秒、を繰り返す。
                # sole_set は使わない(波の段は WAVE_STEPS で決まっている)。
                for step in s:
                    _apply(set(step))
                    time.sleep(w_iv)
            else:
                iv = BLINK_INTERVAL[m]
                _apply(s)
                time.sleep(iv)
                _apply(set())
                time.sleep(iv)

#以下化石
def show(list,mode="solid"):
    """ソレノイド番号(1〜6)と挙上モードを指定する。"""
    for a in isActive:
        _pins[a-1].off()
    isActive.clear()

    # 実践挙上,solid
    if mode=="solid":
        for i in list:
            _pins[i-1].on()
            time.sleep(0.001)
            print(f"{GPIOS[i-1]}がonになった!")
            isActive.append(i)

    # 点滅挙上,blink
    elif mode.startswith("blink"):
        interval = BLINK_INTERVAL[mode]
        for _ in range(2):
            for i in list:
                _pins[i-1].on()
                # print(f"{GPIOS[i-1]}がonになった")
                isActive.append(i)
            time.sleep(interval)

            for j in list:
                _pins[j-1].off()
                # print(f"{GPIOS[j-1]}がoffになった")
                isActive.remove(j)
            time.sleep(interval)

def blink_all_for(seconds):
    """全ソレノイドを seconds 秒間点滅させる (到着時用)"""
    for _ in range(seconds):
        LowAll()
        show([1,2,3,4,5,6],"blink")
        time.sleep(1)


if __name__=="__main__":
    solenoid=Display()
    while True:
        solenoid.selftest()
        time.sleep(0.5)