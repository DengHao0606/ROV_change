import rclpy
from rclpy.node import Node
import time
import os
import termios
import struct
import threading
import json
import argparse

from uv_msgs.msg import CabinState
from uv_msgs.msg import PidParameters
from uv_msgs.msg import PidParametersSum
from uv_msgs.msg import PropellerThrust
from uv_msgs.msg import RobotAxis
from uv_msgs.msg import WorkState

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

# magic number for uart
SETTINGS = [
    0, 0, 6322, 0, 4098, 4098,
    [
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', 1, 254, b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00'
    ]
]


class PID:
    def __init__(self, tty, save_path):

        self.x = PidParameters()
        self.y = PidParameters()
        self.z = PidParameters()
        self.roll = PidParameters()
        self.yaw = PidParameters()
        self.pitch = PidParameters()

        self.path = save_path
        self.tty_writer = tty

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

    def rec(self, data):
        if data.aix == 1:
            self.x.p = data.p
            self.x.i = data.i
            self.x.d = data.d
            self.x.i_limit = data.i_limit
        elif data.aix == 2:
            self.y.p = data.p
            self.y.i = data.i
            self.y.d = data.d
            self.y.i_limit = data.i_limit
        elif data.aix == 3:
            self.z.p = data.p
            self.z.i = data.i
            self.z.d = data.d
            self.z.i_limit = data.i_limit
        elif data.aix == 4:
            self.roll.p = data.p
            self.roll.i = data.i
            self.roll.d = data.d
            self.roll.i_limit = data.i_limit
        elif data.aix == 5:
            self.pitch.p = data.p
            self.pitch.i = data.i
            self.pitch.d = data.d
            self.pitch.i_limit = data.i_limit
        elif data.aix == 6:
            self.yaw.p = data.p
            self.yaw.i = data.i
            self.yaw.d = data.d
            self.yaw.i_limit = data.i_limit

    def send(self):

        buff = b"\xfa\xaf\x02" + \
            struct.pack("<Bffff", self.x.aix, self.x.p, self.x.i,
                        self.x.d, self.x.i_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)

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

    def read(self):
        with open(self.path, 'r', encoding='utf-8') as file:
            pid_data = json.load(file)

            self.x.p = pid_data["x"]["p"]
            self.x.i = pid_data["x"]["i"]
            self.x.d = pid_data["x"]["d"]
            self.x.i_limit = pid_data["x"]["i_limit"]
            self.x.aix = pid_data["x"]["num"]

            self.y.p = pid_data["y"]["p"]
            self.y.i = pid_data["y"]["i"]
            self.y.d = pid_data["y"]["d"]
            self.y.i_limit = pid_data["y"]["i_limit"]
            self.y.aix = pid_data["y"]["num"]

            self.z.p = pid_data["z"]["p"]
            self.z.i = pid_data["z"]["i"]
            self.z.d = pid_data["z"]["d"]
            self.z.i_limit = pid_data["z"]["i_limit"]
            self.z.aix = pid_data["z"]["num"]

            self.roll.p = pid_data["roll"]["p"]
            self.roll.i = pid_data["roll"]["i"]
            self.roll.d = pid_data["roll"]["d"]
            self.roll.i_limit = pid_data["roll"]["i_limit"]
            self.roll.aix = pid_data["roll"]["num"]

            self.pitch.p = pid_data["pitch"]["p"]
            self.pitch.i = pid_data["pitch"]["i"]
            self.pitch.d = pid_data["pitch"]["d"]
            self.pitch.i_limit = pid_data["pitch"]["i_limit"]
            self.pitch.aix = pid_data["pitch"]["num"]

            self.yaw.p = pid_data["yaw"]["p"]
            self.yaw.i = pid_data["yaw"]["i"]
            self.yaw.d = pid_data["yaw"]["d"]
            self.yaw.i_limit = pid_data["yaw"]["i_limit"]
            self.yaw.aix = pid_data["yaw"]["num"]

    def save(self):
        with open(self.path, 'w', encoding='utf-8') as file:
            data = self.dic_init()

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


class TtyReader:  # 读串口

    def __init__(self, path):
        self.tty_dev = open(path, 'rb')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)
        self.name = path

    def read(self, length):
        try:
            buff = os.read(self.tty_dev.fileno(), length)
            return True, buff
        except:
            # print("uart read failed")
            return False, b"\x00"

    def close(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")

    def __del__(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")


class TtyWriter:  # 写串口

    def __init__(self, path):
        self.tty_dev = open(path, 'w')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)

    def write(self, data):
        try:
            os.write(self.tty_dev.fileno(), data)
            return True
        except:
            # print("uart writed failed")
            return False

    def close(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")

    def __del__(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")


class CoreNode(Node):
    def __init__(self, name, h750tty, f407tty, pid_path):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        # 创建话题发布 cabin_state ，定义其中的消息类型为 CabinState
        self.cabin_state_pub = self.create_publisher(
            CabinState, "cabin_state", 10)
        # 创建话题发布 propeller_thrust ，定义其中的消息类型为 PropellerThrust
        self.propeller_thrust_pub = self.create_publisher(
            PropellerThrust, "propeller_thrust", 10)
        # 创建话题发布 robot_position ，定义其中的消息类型为 RobotAxis
        self.robot_position_pub = self.create_publisher(
            RobotAxis, "robot_position", 10)
        # 创建话题发布 robot_speed ，定义其中的消息类型为 RobotAxis
        self.robot_speed_pub = self.create_publisher(
            RobotAxis, "robot_speed", 10)
        # 创建话题发布 PidParameters ，定义其中的消息类型为 PidParameters
        self.pid_parameters_pub = self.create_publisher(
            PidParametersSum, "pid_parameters", 10)

        # 创建话题接收 openloop_thrust ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.openloop_thrust_callback, 10)
        # 创建话题接收 servo_control ，定义其中的消息类型为 CabinState
        self.create_subscription(
            CabinState, 'servo_control', self.servo_control_callback, 10)
        # 创建话题接收 pid_set ，定义其中的消息类型为 PidParameters
        self.create_subscription(
            PidParameters, 'pid_set', self.pid_set_callback, 10)
        # 创建话题接收 work_state ，定义其中的消息类型为 WorkState
        self.create_subscription(
            WorkState, 'work_state', self.work_state_callback, 10)

        # 对F407读写
        self.usb0_reader = TtyReader(f407tty)
        self.usb0_writer = TtyWriter(f407tty)
        # 对H743读写
        self.usb1_reader = TtyReader(h750tty)
        self.usb1_writer = TtyWriter(h750tty)
        # pid参数
        self.pid = PID(self.usb1_writer, pid_path)

    def openloop_thrust_callback(self, data):
        buff = b"\xfa\xaf\x05" + \
            struct.pack("<ffffff", data.x, data.y, data.z,
                        data.roll, data.yaw, data.pitch)+b"\xfb\xbf"
        self.usb1_writer.write(buff)

    def servo_control_callback(self, data):
        buff = b"\xfa\xaf\x06"+struct.pack("<BBBBBB", 0, 0, 0, 0, int(
            data.servo[0]*255), int(data.servo[1]*255))+b"\xfb\xbf"
        self.usb0_writer.write(buff)

    def pid_set_callback(self, data):  # pid参数设置
        self.pid.rec(data)
        # 参数下行
        self.pid.send()
        # 参数保存
        self.pid.save()
        self.get_logger().info("已保存PID参数信息")

    def work_state_callback(self, data):
        buff = b"\xfa\xaf\x01"+struct.pack("<Bffffffffffff", data.state, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)+b"\xfb\xbf"
        self.usb1_writer.write(buff)

    def parameters_init(self):
        self.pid.read()  # pid参数储存路径
        # pid参数下行
        self.pid.send()
        self.get_logger().info("pid 参数已写入")


def TTyRead(tty_reader, node, name):

    cabin_state = CabinState()
    propeller_thrust = PropellerThrust()
    robot_position = RobotAxis()
    robot_speed = RobotAxis()

    node.get_logger().info("%s串口监听已启动" % name)

    while rclpy.ok():
        s, buff = tty_reader.read(1)
        # 校验帧头
        if s and buff == b"\xfa":
            s, buff = tty_reader.read(1)
            if s and buff == b"\xaf":
                count, buff = 0, bytes()
                while rclpy.ok():
                    count += 1
                    s, b = tty_reader.read(1)
                    if s:
                        buff += b
                        # 校验帧尾
                        if len(buff) > 1 and buff[-1] == 0xbf and buff[-2] == 0xfb:
                            # 接收成功后解码
                            # 舱内参数
                            if buff[0] == 0x06 and len(buff[1:-2]) == 6*1:
                                temp_int, hum_int, cabin_state.leak, voltage_int, servo0_int, servo1_int = struct.unpack(
                                    "<BBBBBB", buff[1:-2])
                                cabin_state.temp, cabin_state.hum, cabin_state.voltage, cabin_state.servo[0], cabin_state.servo[
                                    1] = temp_int/1.0, hum_int/255, 12*voltage_int/255, servo0_int/255, servo1_int/255+0.2
                                node.cabin_state_pub.publish(cabin_state)
                                node.get_logger().info("接收到舱内状态信息")
                            # 位置速度信息
                            elif buff[0] == 0x01 and len(buff[1:-2]) == 12*4+1:
                                robot_position.x, robot_position.y, robot_position.z, robot_position.roll, robot_position.pitch, robot_position.yaw, robot_speed.x, robot_speed.y, robot_speed.z, robot_speed.roll, robot_speed.pitch, robot_speed.yaw = struct.unpack(
                                    "<ffffffffffff", buff[2:-2])
                                node.robot_position_pub.publish(robot_position)
                                node.robot_speed_pub.publish(robot_speed)
                                node.get_logger().info("接收到位置、姿态、速度信息")
                            # 推力信息
                            elif buff[0] == 0x04 and len(buff[1:-2]) == 6*4:
                                propeller_thrust.thrust = struct.unpack(
                                    "<ffffff", buff[1:-2])
                                node.propeller_thrust_pub.publish(
                                    propeller_thrust)
                                node.get_logger().info("接收到推力信息")
                            # 接收成功后退出
                            buff = bytes()
                            break
                    if count > 200:  # 接收超时后退出
                        buff = bytes()
                        break


def ParameterBroadcast(node):  # 参数广播
    pid = PidParametersSum()
    while rclpy.ok():
        pid.x[0] =  node.pid.x.p
        pid.x[1] =  node.pid.x.i
        pid.x[2] =  node.pid.x.d
        pid.x[3] =  node.pid.x.i_limit

        pid.y[0] =  node.pid.y.p
        pid.y[1] =  node.pid.y.i
        pid.y[2] =  node.pid.y.d
        pid.y[3] =  node.pid.y.i_limit

        pid.z[0] =  node.pid.z.p
        pid.z[1] =  node.pid.z.i
        pid.z[2] =  node.pid.z.d
        pid.z[3] =  node.pid.z.i_limit

        pid.yaw[0] =  node.pid.yaw.p
        pid.yaw[1] =  node.pid.yaw.i
        pid.yaw[2] =  node.pid.yaw.d
        pid.yaw[3] =  node.pid.yaw.i_limit

        pid.roll[0] =  node.pid.roll.p
        pid.roll[1] =  node.pid.roll.i
        pid.roll[2] =  node.pid.roll.d
        pid.roll[3] =  node.pid.roll.i_limit

        pid.pitch[0] =  node.pid.pitch.p
        pid.pitch[1] =  node.pid.pitch.i
        pid.pitch[2] =  node.pid.pitch.d
        pid.pitch[3] =  node.pid.pitch.i_limit

        node.pid_parameters_pub.publish(pid)
        time.sleep(0.05)


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', nargs='+', type=str, default=['/home/agx/Workspace/Rosws/core_config.json'], help='核心节点参数路径')
    opt = parser.parse_args()

    # 加载配置文件
    with open(opt.cfg[0], 'r', encoding='utf-8') as file:
        config = json.load(file)

    rclpy.init(args=args)  # 初始化rclpy
    node = CoreNode("uv_core", config["h750_path"],
                    config["f407_path"], config["pid_path"])  # 新建一个节点

    node.parameters_init()  # 写入参数

    # 创建串口接收线程
    thread_usb0_read = threading.Thread(
        target=TTyRead, args=(node.usb0_reader, node, "ttyUSB0"))
    thread_usb1_read = threading.Thread(
        target=TTyRead, args=(node.usb1_reader, node, "ttyUSB1"))
    # 创建参数发布线程
    thread_parameter_broadcast = threading.Thread(
        target=ParameterBroadcast, args=(node,))

    # 启动线程
    thread_usb0_read.start()
    thread_usb1_read.start()
    thread_parameter_broadcast.start()

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
