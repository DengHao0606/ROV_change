import rclpy
from rclpy.node import Node
import time
import os
import termios
import threading
import curses
import sys

from uv_msgs.msg import CabinState
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


class PannelNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.cabin_state = CabinState()
        self.propeller_thrust = PropellerThrust()
        self.robot_position = RobotAxis()
        self.robot_speed = RobotAxis()
        self.work_state = WorkState()

        # 创建话题接收 cabin_state ，定义其中的消息类型为 CabinState
        self.create_subscription(
            CabinState, 'cabin_state', self.cabin_state_callback, 10)

        # 创建话题接收 propeller_thrust ，定义其中的消息类型为 PropellerThrust
        self.create_subscription(
            PropellerThrust, 'propeller_thrust', self.propeller_thrust_callback, 10)

        # 创建话题接收 robot_position ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'robot_position', self.robot_position_callback, 10)

        # 创建话题接收 robot_speed ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'robot_speed', self.robot_speed_callback, 10)

        # 创建话题发布 openloop_thrust ，定义其中的消息类型为 RobotAxis
        self.openloop_thrust_pub = self.create_publisher(
            RobotAxis, "openloop_thrust", 10)

        # 创建话题发布 servo_control ，定义其中的消息类型为 CabinState
        self.servo_control_pub = self.create_publisher(
            CabinState, "servo_control", 10)

        # 创建话题发布 work_state ，定义其中的消息类型为 WorkState
        self.work_state_pub = self.create_publisher(
            WorkState, "work_state", 10)

    def cabin_state_callback(self, data):
        self.cabin_state = data

    def propeller_thrust_callback(self, data):
        self.propeller_thrust = data

    def robot_position_callback(self, data):
        self.robot_position = data

    def robot_speed_callback(self, data):
        self.robot_speed = data


def keyboard_read(node):

    # 初始化curses
    screen = curses.initscr()
    # 设置不回显
    curses.noecho()
    # 设置不需要按回车立即响应
    curses.cbreak()
    # 开启键盘模式
    screen.keypad(1)
    # 阻塞模式读取0 非阻塞 1
    screen.nodelay(1)

    openloop_thrust = RobotAxis()

    keyboardinput = screen.getch()
    os.system("clear")

    # l_keyboardinput = keyboardinput

    while rclpy.ok():
        # 读键盘
        keyboardinput = -1
        keyboardinput = screen.getch()

        # if l_keyboardinput != keyboardinput:
        if keyboardinput == -1:
            openloop_thrust.x = 0.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 119:  # W
            openloop_thrust.x = 0.0
            openloop_thrust.y = 200.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 115:  # S
            openloop_thrust.x = 0.0
            openloop_thrust.y = -200.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 97:  # A
            openloop_thrust.x = 100.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 100:  # D
            openloop_thrust.x = -100.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 113:  # Q
            openloop_thrust.x = 0.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = -50.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 101:  # E
            openloop_thrust.x = 0.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 0.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 50.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 105:  # I
            openloop_thrust.x = 0.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = -200.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        elif keyboardinput == 107:  # K
            openloop_thrust.x = 0.0
            openloop_thrust.y = 0.0
            openloop_thrust.z = 200.0
            openloop_thrust.roll = 0.0
            openloop_thrust.pitch = 0.0
            openloop_thrust.yaw = 0.0
            node.openloop_thrust_pub.publish(openloop_thrust)

        if keyboardinput == 49:  # 1
            node.work_state.state = 0
            node.work_state_pub.publish(node.work_state)

        elif keyboardinput == 50:  # 2
            node.work_state.state = 1
            node.work_state_pub.publish(node.work_state)

        elif keyboardinput == 51:  # 3
            node.work_state.state = 2
            node.work_state_pub.publish(node.work_state)

        elif keyboardinput == 52:  # 4
            node.work_state.state = 3
            node.work_state_pub.publish(node.work_state)

        time.sleep(0.05)
        # l_keyboardinput = keyboardinput


def pannel(node):
    buff = "\r\n"
    l_buff = buff
    while rclpy.ok():

        buff = "\r\n"
        buff += "cabin      state :    Temp : {:.1f}℃  Hum : {:.2f}  Leak : {:d}  Voltage : {:.1f}V \r\n".format(
            node.cabin_state.temp, node.cabin_state.hum, node.cabin_state.leak, node.cabin_state.voltage)
        buff += "work       state :    servo1 : {:.2f}   servo2 : {:.2f}   state : {:d}\r\n".format(
            node.cabin_state.servo[0], node.cabin_state.servo[1],node.work_state.state)
        buff += "                      motor1 : {:d}   motor2 : {:d}   motor3 : {:d}\r\n".format(
            int(node.propeller_thrust.thrust[0]), int(node.propeller_thrust.thrust[1]), int(node.propeller_thrust.thrust[2]))
        buff += "                      motor4 : {:d}   motor5 : {:d}   motor6 : {:d}\r\n".format(
            int(node.propeller_thrust.thrust[3]), int(node.propeller_thrust.thrust[4]), int(node.propeller_thrust.thrust[5]))
        buff += "robot coordinate :      X :  {:.2f}       Y :  {:.2f}     Z : {:.2f}\r\n".format(
            node.robot_position.x, node.robot_position.x, node.robot_position.x)
        buff += "                 :   Roll :  {:.2f}   Pitch :  {:.2f}   Yaw :  {:.2f}\r\n".format(
            node.robot_position.roll, node.robot_position.pitch, node.robot_position.yaw)
        buff += "robot      Speed :      X :  {:.2f}       Y :  {:.2f}     Z :  {:.2f}\r\n".format(
            node.robot_speed.x, node.robot_speed.x, node.robot_speed.x)
        buff += "                 :   Roll :  {:.2f}   Pitch :  {:.2f}   Yaw :  {:.2f}\r\n".format(
            node.robot_speed.roll, node.robot_speed.pitch, node.robot_speed.yaw)
        buff += "\r\n"

        if l_buff != buff:
            os.system("clear")
            sys.stdout.write(buff)
            time.sleep(0.1)
        
        l_buff = buff


def main(args=None):

    rclpy.init(args=args)  # 初始化rclpy
    node = PannelNode("uv_cmd_pannel")  # 新建一个节点

    # 创建键盘监听线程
    thread_keyboard_read = threading.Thread(target=keyboard_read, args=(node,))
    # 创建面板显示线程
    thread_pannel = threading.Thread(target=pannel, args=(node,))

    # 启动线程
    thread_keyboard_read.start()
    thread_pannel.start()

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
