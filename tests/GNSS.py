"""u-blox GNSS (I2C) から現在位置を取得する。

元のコードの問題:
  - getPos() を呼ぶたびに設定送信・UBXReader再生成をしていた
  - 呼ぶ間隔(数秒)の間にレシーバのバッファに古いメッセージが溜まり、「古い位置」を返していた
  - fix していなくても lat=0, lon=0 を返し得た / fix を待ち続けて戻ってこなかった
→ 専用スレッドで常に読み続け、main は get() で「最新の有効な位置」を即座に受け取る。
"""
import datetime
import os
import threading
import time

from smbus2 import SMBus, i2c_msg
from pyubx2 import UBXReader, UBX_PROTOCOL, UBXMessage, SET_LAYER_RAM, SET_LAYER_BBR

I2C_BUS = 1
I2C_ADDR = 0x42
MAX_HACC_M = 50.0     # 水平精度の推定がこれ[m]より悪い測位は捨てる (屋内の不正確な位置で再ルートが多発するため)


GNSS_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs", "GNSS.txt")


def _write_log(fix):
    """受信した位置をファイルに追記する。失敗しても読み取りスレッドは止めない"""
    try:
        os.makedirs(os.path.dirname(GNSS_LOG), exist_ok=True)
        t = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S]    ")
        with open(GNSS_LOG, "a", encoding="utf-8") as f:
            f.write(f"{t}lat={fix['lat']} lon={fix['lon']} hAcc={fix['hacc_m']:.1f}m\n")
    except Exception:
        pass


class I2CStream:
    """UBXReader が read(n) できるように I2C をストリーム化する (内部バッファ付き・n バイト揃うまで待つ)"""

    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr
        self.buf = bytearray()

    def _fill(self):
        w = i2c_msg.write(self.addr, [0xFD])    # レジスタ0xFDを読ませるよう要求
        r = i2c_msg.read(self.addr, 2)
        self.bus.i2c_rdwr(w, r)
        d = list(r)
        avail = (d[0] << 8) | d[1]
        if avail == 0 or avail == 0xFFFF:
            return False
        r = i2c_msg.read(self.addr, min(avail, 128))
        self.bus.i2c_rdwr(r)
        self.buf += bytes(r)
        return True

    def read(self, n=1):
        t0 = time.monotonic()
        while len(self.buf) < n:
            if not self._fill():
                if time.monotonic() - t0 > 1.0:
                    break
                time.sleep(0.01)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out


class GNSSReader:
    def __init__(self):
        self._bus = SMBus(I2C_BUS)
        cfg_data = [
            ("CFG_MSGOUT_UBX_NAV_PVT_I2C", 1),  # 位置情報のUBX出力を有効化
            ("CFG_MSGOUT_UBX_NAV_SAT_I2C", 0),  # 衛星情報の提供を無効化
            ("CFG_I2COUTPROT_NMEA", 0),         # NMEAでの出力を無効化
            ("CFG_I2COUTPROT_UBX", 1),          # UBXでわたせを有効化
        ]

        msg = UBXMessage.config_set(SET_LAYER_RAM | SET_LAYER_BBR, 0, cfg_data)
        raw = msg.serialize()
        self._bus.write_i2c_block_data(I2C_ADDR, raw[0], list(raw[1:]))

        self._lock = threading.Lock()
        self._fix = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        ubr = UBXReader(I2CStream(self._bus, I2C_ADDR), protfilter=UBX_PROTOCOL)
        while True:
            try:
                _, p = ubr.read()
            except Exception:
                time.sleep(0.2)
                continue
            if p is None:
                time.sleep(0.05)
                continue
            if p.identity != "NAV-PVT":
                continue
            ok = p.fixType >= 2 and getattr(p, "gnssFixOK", 1)
            if not ok or p.hAcc / 1000.0 > MAX_HACC_M:
                continue
            with self._lock:
                self._fix = dict(
                    lat=p.lat, lon=p.lon,
                    hacc_m=p.hAcc / 1000.0,            # 水平精度推定 [m]
                    speed_mps=p.gSpeed / 1000.0,
                    course_deg=p.headMot,              # 進行方位 (歩行中のみ有効。予備として利用可)
                    num_sv=p.numSV,
                    t=time.monotonic(),
                )
                fix = self._fix
            _write_log(fix)

    def get(self, max_age=5.0):
        """最新の有効な位置 dict を返す。fix が無い/max_age 秒より古ければ None (ブロックしない)"""
        with self._lock:
            f = self._fix
        if f is None or time.monotonic() - f["t"] > max_age:
            return None
        return f