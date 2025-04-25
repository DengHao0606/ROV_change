import os
import json
import time
import struct

from uv_msgs.msg import PidParameters


def dic_init():  # 用于输出一个字典对象
    pid_dic = {
        "x": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 1,
            "name": "x"
        },
        "y": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 2,
            "name": "y"
        },
        "z": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 3,
            "name": "z"
        },
        "roll": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 4,
            "name": "roll"
        },
        "pitch": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 5,
            "name": "pitch"
        },
        "yaw": {
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "i_limit": 100.0,
            "num": 6,
            "name": "yaw"
        }
    }
    return pid_dic


class pid:
    def __init__(self, tty):

        self.x = PidParameters()
        self.y = PidParameters()
        self.z = PidParameters()
        self.roll = PidParameters()
        self.yaw = PidParameters()
        self.pitch = PidParameters()

        self.tty_writer = tty

    def rec(self,data):
        if data.aix == 1:
            self.pid.x.p = data.p
            self.pid.x.i = data.i
            self.pid.x.d = data.d
            self.pid.x.i_limit = data.i_limit
        elif data.aix == 2:
            self.pid.y.p = data.p
            self.pid.y.i = data.i
            self.pid.y.d = data.d
            self.pid.y.i_limit = data.i_limit
        elif data.aix == 3:
            self.pid.z.p = data.p
            self.pid.z.i = data.i
            self.pid.z.d = data.d
            self.pid.z.i_limit = data.i_limit
        elif data.aix == 4:
            self.pid.roll.p = data.p
            self.pid.roll.i = data.i
            self.pid.roll.d = data.d
            self.pid.roll.i_limit = data.i_limit
        elif data.aix == 5:
            self.pid.pitch.p = data.p
            self.pid.pitch.i = data.i
            self.pid.pitch.d = data.d
            self.pid.pitch.i_limit = data.i_limit
        elif data.aix == 6:
            self.pid.yaw.p = data.p
            self.pid.yaw.i = data.i
            self.pid.yaw.d = data.d
            self.pid.yaw.i_limit = data.i_limit


    def send(self):

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.x.aix, self.x.p, self.x.i,
                        self.x.d, self.x.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.y.aix, self.y.p, self.y.i,
                        self.y.d, self.y.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.z.aix, self.z.p, self.z.i,
                        self.z.d, self.z.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.roll.aix, self.roll.p,
                        self.roll.i, self.roll.d, self.roll.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        buff = b"\xfa\xaf\x02" + struct.pack("<Bffff", self.pitch.aix, self.pitch.p,
                                             self.pitch.i, self.pitch.d, self.pitch.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.yaw.aix, self.yaw.p,
                        self.yaw.i, self.yaw.d, self.yaw.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

    def read(self, read_path):
        with open(read_path, 'r', encoding='utf-8') as file:
            pid_data = json.load(file)

            self.x.p = pid_data["x"]["p"]
            self.x.i = pid_data["x"]["i"]
            self.x.d = pid_data["x"]["d"]
            self.x.i_limit = pid_data["x"]["i_limit"]

            self.y.p = pid_data["y"]["p"]
            self.y.i = pid_data["y"]["i"]
            self.y.d = pid_data["y"]["d"]
            self.y.i_limit = pid_data["y"]["i_limit"]

            self.z.p = pid_data["z"]["p"]
            self.z.i = pid_data["z"]["i"]
            self.z.d = pid_data["z"]["d"]
            self.z.i_limit = pid_data["z"]["i_limit"]

            self.roll.p = pid_data["roll"]["p"]
            self.roll.i = pid_data["roll"]["i"]
            self.roll.d = pid_data["roll"]["d"]
            self.roll.i_limit = pid_data["roll"]["i_limit"]

            self.pitch.p = pid_data["pitch"]["p"]
            self.pitch.i = pid_data["pitch"]["i"]
            self.pitch.d = pid_data["pitch"]["d"]
            self.pitch.i_limit = pid_data["pitch"]["i_limit"]

            self.yaw.p = pid_data["yaw"]["p"]
            self.yaw.i = pid_data["yaw"]["i"]
            self.yaw.d = pid_data["yaw"]["d"]
            self.yaw.i_limit = pid_data["yaw"]["i_limit"]

    def save(self, save_path):
        with open(save_path, 'w', encoding='utf-8') as file:
            data = dic_init()

            data["x"]["p"] = self.x.p
            data["x"]["i"] = self.x.i
            data["x"]["d"] = self.x.d
            data["x"]["i_limit"] = self.x.i_limit

            data["y"]["p"] = self.y.p
            data["y"]["i"] = self.y.i
            data["y"]["d"] = self.y.d
            data["y"]["i_limit"] = self.y.i_limit

            data["z"]["p"] = self.z.p
            data["z"]["i"] = self.z.i
            data["z"]["d"] = self.z.d
            data["z"]["i_limit"] = self.z.i_limit

            data["roll"]["p"] = self.roll.p
            data["roll"]["i"] = self.roll.i
            data["roll"]["d"] = self.roll.d
            data["roll"]["i_limit"] = self.roll.i_limit

            data["pitch"]["p"] = self.pitch.p
            data["pitch"]["i"] = self.pitch.i
            data["pitch"]["d"] = self.pitch.d
            data["pitch"]["i_limit"] = self.pitch.i_limit

            data["yaw"]["p"] = self.yaw.p
            data["yaw"]["i"] = self.yaw.i
            data["yaw"]["d"] = self.yaw.d
            data["yaw"]["i_limit"] = self.yaw.i_limit

            json.dump(data, file)
