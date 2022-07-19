from cflib.crtp import RadioDriver
import struct
from ctypes import *


class olsrPacketHeader_t(Structure):
    _pack_ = 1
    _fields_ = [
        ('sender', c_uint8),
        ('seqNumber', c_int16)
    ]


class olsrUnit_t(Structure):
    _pack_ = 1
    _fields_ = [
        ('neighbor1', c_uint8),
        ('neighbor2', c_uint8)
    ]


class olsrPacketPayload_t(Structure):
    _pack_ = 1
    _fields_ = [
        ('units', olsrUnit_t * 3)
    ]


class olsrPacketPacket_t(Structure):
    _pack_ = 1
    _fields_ = [
        ('header', olsrPacketHeader_t),
        ('payload', olsrPacketPayload_t)
    ]

    def __str__(self):
        print(self._fields_[0][0], self._fields_[1][0])


uri = "radio://0/4/2M/E7E7E7E7E7"


def convert_crtp(ctypes: list, pattern: bytearray, data_size: int) -> list:
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
            olsrPacket = olsrPacketPacket_t.from_buffer(pk.data, 0)
            # print(sizeof(olsrPacket))
            print(olsrPacket.header.sender, olsrPacket.header.seqNumber)
            print(olsrPacket.payload.units[0].neighbor1, olsrPacket.payload.units[0].neighbor2)
            print(olsrPacket.payload.units[1].neighbor1, olsrPacket.payload.units[1].neighbor2)
            print(olsrPacket.payload.units[2].neighbor1, olsrPacket.payload.units[2].neighbor2)
            print("===================================")
