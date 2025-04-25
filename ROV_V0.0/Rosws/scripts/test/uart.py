#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#                              _ooOoo_
#                             o8888888o
#                             88" . "88
#                             (| -_- |)
#                              O\ = /O
#                           ____/`---'\____
#                        .   ' \\| |// `.
#                         / \\||| : |||// \
#                        / _||||| -:- |||||- \
#                         | | \\\ - /// | |
#                       | \_| ''\---/'' | |
#                        \ .-\__ `-` ___/-. /
#                    ___`. .' /--.--\ `. . __
#                  ."" '< `.___\_<|>_/___.' >'"".
#                 | | : `- \`.;`\ _ /`;.`/ - ` : | |
#                    \ \ `-. \_ __\ /__ _/ .-` / /
#           ======`-.____`-.___\_____/___.-`____.-'======
#                              `=---='
#
#           .............................................
#                     佛祖保佑             永无BUG


import os
import termios

#magic number
SETTINGS = [
    0, 0, 6322, 0, 4098, 4098,
    [
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', 1, 254, b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00'
    ]
]

class Reader:

    def __init__(self, path):
        self.tty_dev = open(path, 'rb')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)

    def read(self,length):
        try :
            buff =  os.read(self.tty_dev.fileno(), length)
            return True , buff
        except :
            #print("uart read failed")
            return False , b"\x00"
            
    def close(self):
        try :
            self.tty_dev.close()
        except :
            print("uart has been already closed")

    def __del__(self):
        try :
            self.tty_dev.close()
        except :
            print("uart has been already closed")

class Writer:

    def __init__(self, path):
        self.tty_dev = open(path, 'w')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)

    def write(self,data):
        try :
            os.write(self.tty_dev.fileno(), data)
            return True
        except :           
            # print("uart writed failed")
            return False

    def close(self):
        try :
            self.tty_dev.close()
        except :
            print("uart has been already closed")

    def __del__(self):
        try :
            self.tty_dev.close()
        except :
            print("uart has been already closed")




