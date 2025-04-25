#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#

import uart
import struct
import time

serial_reader = uart.Reader("/dev/ttyUSB0")
# serial0_writer = uart.Writer("/dev/ttyUSB0")
# serial1_writer = uart.Writer("/dev/ttyUSB1")
count = 0

while True:
    s, b = serial_reader.read(1)
    if s and b == b"\xfa":
        s, b = serial_reader.read(1)
        if s and b == b"\xaf":
            # s, b = serial_reader.read(1)
            # if s and b == b"\x06":
            buff = b"\xfa\xaf"
            while True:
                s, b = serial_reader.read(1)
                if s:
                    buff += b
                    # 校验帧尾
                    if len(buff) > 1 and buff[-1] == 0xbf and buff[-2] == 0xfb:
                        print(buff)
                        print(len(buff))
                        break
            break
# while True:
#     buff = b"\xfa\xaf\x05"+struct.pack("<ffffff",0,0,0,0,0,0)+b"\xfb\xbf"
#     print(buff)
#     serial0_writer.write(buff) 
#     serial1_writer.write(buff) 
#     time.sleep(1)