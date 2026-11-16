"""BNO055 の診断とキャリブレーション保存ツール。単体で実行する: python calibrate_bno055.py

1. mode が 12 (NDOF) か確認する (8=IMUPLUS だと地磁気を使わず「起動時の向きが0°」になる)
2. センサを手に持って 8 の字に振る (磁気=mag を 3 にする) / 静止 (gyro) / 6面を静止 (accel)
3. sys / gyro / accel / mag の4つすべてが 3 になったらオフセットを bno055_calib.json に保存する
   → 以降 userDirection.py が起動時に自動で復元する
4. 表示される heading が、スマホのコンパス等の北と一致しているか確認する
"""
import json
import os
import time

import serial
import adafruit_bno055 as bno055

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bno055_calib.json")

uart = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.1)
s = bno055.BNO055_UART(uart)
s.mode = bno055.NDOF_MODE
time.sleep(0.1)
print(f"mode = {s.mode}  (12=NDOF 絶対方位 / 8=IMUPLUS 相対方位)")

while True:
    try:
        cal = s.calibration_status          # (sys, gyro, accel, mag)
        e = s.euler
        print(f"sys/gyro/accel/mag = {cal}   heading={e[0] if e else None}")
        if all(v == 3 for v in cal):        # sys, gyro, accel, mag の4つすべてが3になってから保存する
            time.sleep(2)
            s.mode = bno055.CONFIG_MODE
            time.sleep(0.05)
            data = dict(acc=list(s.offsets_accelerometer), gyro=list(s.offsets_gyroscope),
                        mag=list(s.offsets_magnetometer),
                        racc=s.radius_accelerometer, rmag=s.radius_magnetometer)
            with open(CALIB_FILE, "w") as f:
                json.dump(data, f)
            s.mode = bno055.NDOF_MODE
            print("保存しました:", CALIB_FILE, data)
            break
    except Exception as ex:
        print("read error:", ex)
    time.sleep(0.5)