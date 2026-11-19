import time
import threading
from math import random
from gpiozero import LED
import argparse

def parse_args(argv=None):
    """コマンドライン引数を読み取る。--pin-mode 2x2 / 2x3（既定は 2x3）"""
    parser = argparse.ArgumentParser(description="点字ピン方向定位デバイス（2x2 / 2x3 ピンモード切替）")
    parser.add_argument("mode", choices=["1","2","3","4","5","6","mae","mmae","migi","mback","back","hback","hidari","hmae","random1"], default="random1",
                        help="実行するもの: 上げるピン番号 or 上げるパターン or 一連のテスト")
    return parser.parse_args(argv)

# SDA:pin2  SCL:pin3  TXD:pin14  RXD:pin15
GPIOS=[23,24,25,8,7,1,12,16,20]
DELAY=3

_pins = [LED(g, initial_value=False) for g in GPIOS]

def show(num,delay):
    _pins[num].on()
    time.sleep(delay)
    _pins[num].on()


def first():
    for count in range(2):
        list=[0,1,2,3,4,5]
        for block in range(6):
            if (block==0) and (count!=0):
                ex=r
                r=random.randint(0,len(list)-1)
                while ex==r:
                    r=random.randint(0,len(list)-1)
            else:
                r=random.randint(0,len(list)-1)
            print(r)
            list.pop(r)
            show(r,3)

class ascendSoles():
    def __init__(self):
        self.pins=None
    def a():
        show([1],3)
    def b():
        show([2],3)
    def c():
        show([3],3)
    def d():
        show([4],3)
    def e():
        show([5],3)
    def f():
        show([6],3)
    def


if __name__=="__main__":
    args=parse_args()
    if args.mode=="1" or "2" or "3" or "4" or "5" or "6":
        show([int(args.mode)],DELAY)
    elif args.mode=="mae":
        show([1,4],DELAY)
    elif args.mode=="back":
        show([3,6],DELAY)
        