"""BNO055 (UART) から、校正を適用した「磁北基準」の方位を取得する。

- 0° = 磁北、時計回りに増加 (真北ではない。磁気偏角の補正はしていない)
- 起動時に、calibrateBNO055.py が保存した bno055_calib.json を読み込んでセンサーに書き込む
  (BNO055は電源を入れ直すたび、また初期化のたびに校正値が失われるため)
- 方位の安定化:
    ・起動直後は方位が落ち着くまで待つ (wait_settled)
    ・数回の読み取りを「円周平均」で1つにまとめ、ばらつき(円周標準偏差)も返す (get_detail)
    ・磁気の校正状態 (mag_cal) とばらつきから、値を信用してよいかの目安を返す

main.py からの使い方 (従来と同じ):
    from userDirection import HeadingReader
    head = HeadingReader()
    hd = head.get()                 # (heading, mag_cal) または None
    if hd is not None:
        heading, mag_cal = hd       # mag_cal: 磁気センサーの校正状態 0〜3

詳しい情報が必要なとき:
    d = head.get_detail()           # dict(heading, mag_cal, cal, std_deg, n)
"""
import json
import math
import os
import struct
import time

import serial
import adafruit_bno055

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bno055_calib.json")
UART_PORT = "/dev/serial0"

# 校正値のレジスタ (データシート 4.3.85 / 4.3.91 / 4.3.97 / 4.3.103 / 4.3.105)
REG_ACC_OFFSET = 0x55
REG_MAG_OFFSET = 0x5B
REG_GYR_OFFSET = 0x61
REG_ACC_RADIUS = 0x67
REG_MAG_RADIUS = 0x69

MODES = {"ndof": adafruit_bno055.NDOF_MODE, "fmc_off": getattr(adafruit_bno055, "NDOF_FMC_OFF_MODE", 0x0B)}
BPS=115200
ARGUMENT_DEG=-7.2  # 柏だから東経7.2

# ---------------------------------------------------------------
# 角度の統計 (円周平均・円周標準偏差)。 359°と1°の平均を、180°ではなく0°にするため
# ---------------------------------------------------------------
def circular_mean_std(angles_deg):
    """角度[deg]のリストから (円周平均[0,360), 円周標準偏差[deg]) を返す。
    平均: 単位ベクトルの和の向き atan2(Σsin, Σcos)。 標準偏差: sqrt(-2 ln R) (Rは平均ベクトルの長さ)"""
    a = [math.radians(x) for x in angles_deg]
    n = len(a)
    s = sum(math.sin(x) for x in a) / n
    c = sum(math.cos(x) for x in a) / n
    mean = math.degrees(math.atan2(s, c)) % 360.0
    R = min(max(math.hypot(s, c), 1e-12), 1.0)
    return mean, math.degrees(math.sqrt(max(-2.0 * math.log(R), 0.0)))


def angle_diff(a, b):
    """a - b を -180〜180 に収めて返す"""
    return (a - b + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------
# 校正値の読み込みと書き込み
# ---------------------------------------------------------------
def _set_mode(sensor, mode):
    sensor.mode = mode
    time.sleep(0.05)


def _vec3(values):
    v = tuple(int(x) for x in values)
    if len(v) != 3:
        raise ValueError(f"3軸の値が必要です: {values}")
    return v


def load_calibration(path=CALIB_FILE):
    """校正ファイル (calibrateBNO055.py の出力) を読む。なければ、または壊れていれば None"""
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return dict(
            acc=_vec3(d["acc"]),
            gyro=_vec3(d["gyro"]),
            mag=_vec3(d["mag"]),
            racc=int(d["racc"]),
            rmag=int(d["rmag"]),
        )
    except FileNotFoundError:
        print(f"[BNO055] 校正ファイルがありません: {path}  (先に calibrateBNO055.py を実行してください)")
    except (ValueError, KeyError, TypeError) as e:
        print(f"[BNO055] 校正ファイルを読めません: {e}")
    return None


def _write_reg(uart, register, data, retries=5, byte_delay=0.002):
    """UARTでレジスタに書き込む (データシート 4.7 の Register write)。

    ライブラリの書き込みは、コマンドを一度にまとめて送るため、BUS_OVER_RUN_ERROR (7) で
    失敗することがある。ここでは1バイトずつ間隔をあけて送り、失敗したら再試行する。
    (バイト間の待ちは、データシートの上限 30ms より十分短い 2ms)"""
    frame = bytes([0xAA, 0x00, register, len(data)]) + data
    last = None
    for attempt in range(retries):
        uart.reset_input_buffer()
        for b in frame:
            uart.write(bytes([b]))
            time.sleep(byte_delay)
        t0 = time.monotonic()
        while uart.in_waiting < 2 and time.monotonic() - t0 < 0.25:
            time.sleep(0.001)
        resp = uart.read(uart.in_waiting)
        if len(resp) >= 2 and resp[0] == 0xEE and resp[1] == 0x01:      # 0x01 = WRITE_SUCCESS
            return
        last = resp
        time.sleep(0.05 * (attempt + 1))
    raise RuntimeError(f"UART write error (reg 0x{register:02X}): "
                       f"{last.hex() if last else '応答なし'}")


def apply_calibration(sensor, uart, path=CALIB_FILE, mode=adafruit_bno055.NDOF_MODE):
    """校正値をセンサーに書き込む。書き込めたことを読み戻して確認し、成功なら True を返す。
    オフセットの読み書きは、データシートのとおり CONFIG モードで行い、終わったら指定のモードに戻す"""
    c = load_calibration(path)
    if c is None:
        return False

    ok = False
    try:
        _set_mode(sensor, adafruit_bno055.CONFIG_MODE)
        _write_reg(uart, REG_ACC_OFFSET, struct.pack("<hhh", *c["acc"]))
        _write_reg(uart, REG_GYR_OFFSET, struct.pack("<hhh", *c["gyro"]))
        _write_reg(uart, REG_MAG_OFFSET, struct.pack("<hhh", *c["mag"]))
        _write_reg(uart, REG_ACC_RADIUS, struct.pack("<h", c["racc"]))
        _write_reg(uart, REG_MAG_RADIUS, struct.pack("<h", c["rmag"]))
        ok = (tuple(sensor.offsets_accelerometer) == c["acc"]
              and tuple(sensor.offsets_gyroscope) == c["gyro"]
              and tuple(sensor.offsets_magnetometer) == c["mag"]
              and sensor.radius_accelerometer == c["racc"]
              and sensor.radius_magnetometer == c["rmag"])
    except (RuntimeError, OSError) as e:
        print(f"[BNO055] 校正値の書き込みに失敗しました: {e}")
    finally:
        _set_mode(sensor, mode)                          # 必ず絶対方位モードに戻す

    if ok:
        print("[BNO055] 校正値を適用しました")
    else:
        print("[BNO055] 警告: 校正値の読み戻しが一致しません (校正が効いていない可能性)")
    return ok


# ---------------------------------------------------------------
# 方位の読み取り
# ---------------------------------------------------------------
class HeadingReader:
    def __init__(self, port=UART_PORT, calib_file=CALIB_FILE, mode="ndof",
                 burst=5, burst_gap=0.01, settle_timeout=15.0, settle_std_deg=0.5, settle_drift_deg=0.3):
        """
        mode           : "ndof" (既定) または "fmc_off" (NDOF_FMC_OFF。実験用)
        burst          : get() で何回読んで平均するか
        burst_gap      : 読み取りの間隔 [秒]
        settle_timeout : 起動後、方位が落ち着くまで待つ最大秒数 (0なら待たない)
        settle_std_deg : 「落ち着いた」とみなす円周標準偏差の上限 [deg]
        settle_drift_deg: 「落ち着いた」とみなす、窓の前半と後半の平均の差の上限 [deg] (ゆっくり収束中の検出用)
        """
        self._mode = MODES[mode]
        self._uart = serial.Serial(port, BPS, timeout=0.1)
        self.sensor = adafruit_bno055.BNO055_UART(self._uart)      # 初期化でNDOFモードになる (校正値は消える)
        self.calibration_applied = apply_calibration(self.sensor, self._uart, calib_file, self._mode)
        if self.sensor.mode != self._mode:
            _set_mode(self.sensor, self._mode)
        self.burst, self.burst_gap = max(int(burst), 1), burst_gap
        self._last = None
        self.last_std = None
        #print(f"[BNO055] mode = {self.sensor.mode}  (12=NDOF 絶対方位 / 11=NDOF_FMC_OFF)")
        self.settle_result = None
        if settle_timeout > 0:
            self.settle_result = self.wait_settled(settle_timeout, settle_std_deg, drift_deg=settle_drift_deg)

    def close(self):
        try:
            self._uart.close()
        except Exception:
            pass

    # -- 1回分の読み取り (失敗したらバッファを捨てて再試行) --
    def _read_heading_once(self, retries=5):
        for attempt in range(retries):
            try:
                euler = self.sensor.euler                 # (heading, roll, pitch)
                if euler is None or euler[0] is None:
                    raise ValueError("euler が None")
                return float(euler[0])
            except (RuntimeError, ValueError, OSError):
                self._uart.reset_input_buffer()
                time.sleep(0.05 * (attempt + 1))
        return None

    def _read_cal(self):
        try:
            return tuple(self.sensor.calibration_status)  # (sys, gyro, accel, mag)
        except (RuntimeError, ValueError, OSError):
            self._uart.reset_input_buffer()
            return None

    def wait_settled(self, timeout=15.0, max_std_deg=0.5, window=20, interval=0.1, drift_deg=0.3):
        """方位が落ち着くまで待つ。次の3つがそろったら「落ち着いた」とする:
          (1) 直近 window 回の円周標準偏差が max_std_deg 以下 (ばらつきが小さい)
          (2) 窓の前半と後半の平均の差が drift_deg 以下 (ゆっくり動き続けていない)
          (3) 磁気の校正状態が3
        戻り値: dict(ok, waited_s, std_deg, drift_deg, cal)"""
        t0 = time.monotonic()
        buf, cal, std, drift = [], None, None, None
        while time.monotonic() - t0 < timeout:
            h = self._read_heading_once(retries=2)
            if h is not None:
                buf.append(h)
                buf = buf[-window:]
            c = self._read_cal()
            cal = c if c is not None else cal
            if len(buf) >= window:
                _, std = circular_mean_std(buf)
                m1, _ = circular_mean_std(buf[:window // 2])
                m2, _ = circular_mean_std(buf[window // 2:])
                drift = abs(angle_diff(m2, m1))
                if std <= max_std_deg and drift <= drift_deg and cal is not None and cal[3] == 3:
                    res = dict(ok=True, waited_s=time.monotonic() - t0, std_deg=std, drift_deg=drift, cal=cal)

                    #print(f"[BNO055] 方位が落ち着きました ({res['waited_s']:.1f}秒, ばらつき {std:.2f}°, "
                    #    f"変動 {drift:.2f}°, 校正 {cal})")
                    return res
            time.sleep(interval)
        res = dict(ok=False, waited_s=time.monotonic() - t0, std_deg=std, drift_deg=drift, cal=cal)
        """
        print(f"[BNO055] 警告: {timeout:.0f}秒待っても方位が落ち着きませんでした "
              f"(ばらつき {std}, 変動 {drift}, 校正 {cal})。"
              "磁気の乱れ (ソレノイドや配線の電流、鉄) の影響を疑ってください")
            """
        return res

    def get_detail(self):
        """burst回読んで円周平均を返す。読めなければ直近の値(なければNone)。
        dict(heading[deg], mag_cal, cal=(sys,gyro,accel,mag), std_deg, n)"""
        hs = []
        for i in range(self.burst):
            h = self._read_heading_once()
            if h is not None:
                hs.append(h)
            if i + 1 < self.burst:
                time.sleep(self.burst_gap)
        cal = self._read_cal()
        if not hs or cal is None:
            if not hs:
                print("[BNO055] 読み取りに失敗が続きました。直近の値を返します")
            return self._last
        mean, std = circular_mean_std(hs)
        self.last_std = std
        self._last = dict(heading=mean, mag_cal=cal[3], cal=cal, std_deg=std, n=len(hs))
        return self._last

    def get(self):
        """(heading[deg, 磁北基準], mag_cal[0-3]) を返す (従来と同じ形式)。 読めなければ None"""
        d = self.get_detail()
        return None if d is None else (d["heading"], d["mag_cal"])


# ---- 従来の関数形式 (getHeading) も残す。最初に呼ばれたときにだけセンサーを開く ----
_reader = None


def getHeading():
    """方位(度, 磁北基準)だけを返す"""
    global _reader
    if _reader is None:
        _reader = HeadingReader()
    r = _reader.get()
    return None if r is None else r[0]


if __name__ == "__main__":
    reader = HeadingReader()
    while True:
        d = reader.get_detail()
        if d is not None:
            print(f"heading={d['heading']:6.1f}°  ばらつき={d['std_deg']:.2f}°  校正(sys,gyro,acc,mag)={d['cal']}")
        time.sleep(0.5)