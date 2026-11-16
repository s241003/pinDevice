import time
import threading
from math import random
from gpiozero import LED

# SDA:pin2  SCL:pin3  TXD:pin14  RXD:pin15
GPIOS=[23,24,25,8,7,1,12,16,20]
_pins = [LED(g, initial_value=False) for g in GPIOS]

def show(num):
    _pins[num]


def first():
    for count in range(5):
        list=[0,1,2,3,4,5]
        for block in range(6):
            if (block==0) and (count!=0):
                ex=r
                r=random.randint(0,len(list)-1)
                while ex==r:
                    r=random.randint(0,len(list)-1)
            else:
                r=random.randint(0,len(list)-1)
            list.pop(r)
            show(r)

def second():
    for count