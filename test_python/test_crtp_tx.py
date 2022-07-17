import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from cflib.crtp import RadioDriver
from cflib.crtp.crtpstack import CRTPPacket
import struct
from ctypes import *

uri = "radio://0/60/2M/E7E7E7E702"


def convert(ctypes: list, pattern: bytearray) -> list:
    res = []
    start_p = 0
    end_p = 0
    for cur_type in ctypes:
        end_p = end_p + sizeof(cur_type)
        res.append(struct.unpack(cur_type._type_, pattern[start_p:end_p])[0])
        start_p = end_p
    return res


if __name__ == '__main__':
    rd = RadioDriver()
    rd.connect(uri=uri, link_error_callback=None, link_quality_callback=None)
    i = 1
    while True:
        rd.receive_packet()
        pk = rd.in_queue.get(block=True, timeout=None)
        port = (pk.header & 0xF0) >> 4
        channel = pk.header & 0x03
        if port == 9 and channel == 2 and pk.is_data_size_valid():
            res = convert(ctypes=[c_uint8, c_uint16, c_uint16, c_float], pattern=pk.data)
            print(i, res)
            i = i + 1
