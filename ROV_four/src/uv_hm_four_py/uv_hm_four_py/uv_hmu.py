# 水下机器人硬件管理单元
# "Underwater Vehicle Hardware Management Unit



import rclpy
from rclpy.node import Node
import time
import os
import termios
import struct
import threading
import json
import argparse

from uv_msgs.msg import RobotDeviceManager  # 机器人设备管理器
from uv_msgs.msg import RobotMotionController  # 机器人运动控制器



from uv_msgs.msg import PidParams
from uv_msgs.msg import PidControllers
from uv_msgs.msg import PidControllersState

from uv_msgs.msg import ThrustCurve
from uv_msgs.msg import ThrustCurves

from uv_msgs.msg import RobotAxis
from uv_msgs.msg import ServoSet
from uv_msgs.msg import TargetPosDown
from uv_msgs.msg import ImuData

from uv_control_four_py import Pid
from uv_control_four_py import Curve
from uv_control_four_py import Serial

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


class CoreNode(Node):
    def __init__(self, name, h750tty, f407tty, pid_path, curve_path):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.MotionController = RobotMotionController()
        self.DeviceManager = RobotDeviceManager()

        # 话题发布
        # 创建话题发布 motion_controller ，定义其中的消息类型为 RobotMotionController
        self.motion_controller_pub = self.create_publisher(
            RobotMotionController, "motion_controller", 10)
        # 创建话题发布 device_manager ，定义其中的消息类型为 ImuData
        self.device_manager_pub = self.create_publisher(
            RobotDeviceManager, "device_manager", 10)

        # 创建话题发布 pid_controllers ，定义其中的消息类型为 PidParameters
        self.pid_controllers_pub = self.create_publisher(
            PidControllers, "pid_controllers", 10)
        # 创建话题发布 pid_controllers ，定义其中的消息类型为 PidParameters
        self.curves_pub = self.create_publisher(
            ThrustCurves, "curves", 10)

        # 话题接收
        # 创建话题接收 openloop_thrust ，定义其中的消息类型为 RobotAxis
        self.create_subscription(
            RobotAxis, 'openloop_thrust', self.openloop_thrust_callback, 10)
        # 创建话题接收 servo_control ，定义其中的消息类型为 ServoSet
        self.create_subscription(
            ServoSet, 'servo_control', self.servo_control_callback, 10)
        # 创建话题接收 pid_param_set ，定义其中的消息类型为 PidParams
        self.create_subscription(
            PidParams, 'pid_params_set', self.pid_params_set_callback, 10)
        # 创建话题接收 pid_controllers_set ，定义其中的消息类型为 PidControllersState
        self.create_subscription(
            PidControllersState, 'pid_controllers_set', self.pid_controllers_set_callback, 10)
        # 创建话题接收 target_pos_down ，定义其中的消息类型为 TargetPosDown
        self.create_subscription(
            TargetPosDown, 'target_pos_down', self.target_pos_down_callback, 10)
        # 创建话题接收 dvl_set ，定义其中的消息类型为 ImuData
        self.create_subscription(
            ImuData, 'dvl_set', self.dvl_set_callback, 10)
        # 创建话题接收 thrust_curve_set ，定义其中的消息类型为 ThrustCurve
        self.create_subscription(
            ThrustCurve, 'thrust_curve_set', self.thrust_curve_set_callback, 10)

        # 对F407读写
        self.usb0_reader = Serial.TtyReader(f407tty)
        self.usb0_writer = Serial.TtyWriter(f407tty)
        # 对H743读写
        self.usb1_reader = Serial.TtyReader(h750tty)
        self.usb1_writer = Serial.TtyWriter(h750tty)
        # pid参数
        self.pid = Pid.PID(self.usb1_writer, pid_path)
        # 推力曲线
        self.curve = Curve.CURVE(self.usb1_writer, curve_path)

    def openloop_thrust_callback(self, data):
        buff = b"\xfa\xaf\x02" + \
            struct.pack("<ffffff", data.x, data.y, data.z,
                        data.rx, data.ry, data.rz)+b"\xfb\xbf"
        self.usb1_writer.write(buff)

    def servo_control_callback(self, data):
        #print(f"data.angle:{data.angle}, data.num :{data.num}") 
        if data.angle > 1.0:
            data.angle = 1.0
        elif data.angle < 0.0:
            data.angle = 0.0
       
        buff = b"\xfa\xaf\x06" + \
            struct.pack("<Bf", data.num, data.angle)+b"\xfb\xbf"
        self.usb0_writer.write(buff)

    def pid_params_set_callback(self, data):  # pid参数设置
        self.pid.topicrec(data)

        # 参数下行
        self.pid.hwinit()
        # 参数保存
        self.pid.filesave()
        self.get_logger().info("已保存PID参数信息")

    def thrust_curve_set_callback(self, data):
        self.curve.topicrec(data)

        # 参数下行
        self.curve.hwinit()
        # 参数保存
        self.curve.filesave()
        self.get_logger().info("已保存推力曲线")

    def target_pos_down_callback(self, data):  # 设置目标位置
        buff = b"\xfa\xaf\x03"+struct.pack("<Bffffff", data.cs, data.pos.x, data.pos.y,
                                           data.pos.z, data.pos.rx, data.pos.ry, data.pos.rz)+b"\xfb\xbf"
        self.usb1_writer.write(buff)

    def pid_controllers_set_callback(self, data):  # 设置PID控制器工作状态
        buff = b"\xfa\xaf\x01" + \
            struct.pack("<BBBBBB", data.x, data.y, data.z,
                        data.rx, data.ry, data.rz)+b"\xfb\xbf"
        self.usb1_writer.write(buff)

    def dvl_set_callback(self, data):  # 设置PID控制器工作状态
        buff = b"\xfa\xaf\x04" + \
            struct.pack("<B", data.dvl,)+b"\xfb\xbf"
        self.usb0_writer.write(buff)

    def parameters_init(self):
        # pid控制器硬件初始化
        self.pid.hwinit()
        self.get_logger().info("pid 参数已写入")
        # 推力曲线初始化
        self.curve.hwinit()
        self.get_logger().info("推力曲线已写入")


def TTyRead(tty_reader, node, name):
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
                            # 设备管理器
                            if buff[0] == 0x01 and len(buff[1:-2]) == 21:  # 校验长度
                                node.DeviceManager.leak, node.DeviceManager.vol, node.DeviceManager.tem, \
                                    node.DeviceManager.hum, node.DeviceManager.servo[0], node.DeviceManager.servo[1] = struct.unpack(
                                        "<Bfffff", buff[1:-2])
                                node.device_manager_pub.publish(
                                    node.DeviceManager)
                                # node.get_logger().info(name+":接收到设备管理器信息")
                            # 运动控制器
                            elif buff[0] == 0x00 and len(buff[1:-2]) >= 152:  # 校验长度
                                # 机器人位置 1~24
                                node.MotionController.pos.x, node.MotionController.pos.y, node.MotionController.pos.z, \
                                    node.MotionController.pos.rx, node.MotionController.pos.ry, node.MotionController.pos.rz = struct.unpack(
                                        "<ffffff", buff[1:25])
                                # 目标位置(base) 25~48
                                node.MotionController.tpos_inbase.x, node.MotionController.tpos_inbase.y, node.MotionController.tpos_inbase.z, \
                                    node.MotionController.tpos_inbase.rx, node.MotionController.tpos_inbase.ry, node.MotionController.tpos_inbase.rz = struct.unpack(
                                        "<ffffff", buff[25:49])
                                # 目标位置(world) 49~72
                                node.MotionController.tpos_inworld.x, node.MotionController.tpos_inworld.y, node.MotionController.tpos_inworld.z, \
                                    node.MotionController.tpos_inworld.rx, node.MotionController.tpos_inworld.ry, node.MotionController.tpos_inworld.rz = struct.unpack(
                                        "<ffffff", buff[49:73])
                                # PID控制器状态 73~78
                                node.MotionController.pidstate.x, node.MotionController.pidstate.y, node.MotionController.pidstate.z, \
                                    node.MotionController.pidstate.rx, node.MotionController.pidstate.ry, node.MotionController.pidstate.rz, = struct.unpack(
                                        "<BBBBBB", buff[73:79])
                                # IMU数据 79~128
                                node.MotionController.imu.mode, node.MotionController.imu.dvl,\
                                    node.MotionController.imu.pos.x, node.MotionController.imu.pos.y, node.MotionController.imu.pos.z, \
                                    node.MotionController.imu.pos.rx, node.MotionController.imu.pos.ry, node.MotionController.imu.pos.rz, \
                                    node.MotionController.imu.spd.x, node.MotionController.imu.spd.y, node.MotionController.imu.spd.z, \
                                    node.MotionController.imu.spd.rx, node.MotionController.imu.spd.ry, node.MotionController.imu.spd.rz = struct.unpack(
                                        "<BBffffffffffff", buff[79:129])
                                # 推力数据 129~152
                                node.MotionController.thrust.thrust[0], node.MotionController.thrust.thrust[1], node.MotionController.thrust.thrust[2], \
                                    node.MotionController.thrust.thrust[3], node.MotionController.thrust.thrust[4], node.MotionController.thrust.thrust[5], = struct.unpack(
                                        "<ffffff", buff[129:153])
                                node.motion_controller_pub.publish(
                                    node.MotionController)
                                # node.get_logger().info(name+":接收到运动控制器信息")

                            # 接收成功后退出
                            buff = bytes()
                            break
                    if count > 200:  # 接收超时后退出
                        buff = bytes()
                        break



def ParameterBroadcast(node):  # 参数广播
    while rclpy.ok():
        node.pid_controllers_pub.publish(node.pid.pid)
        time.sleep(0.05)
        node.curves_pub.publish(node.curve.curves)
        time.sleep(0.05)


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid-path', nargs='+', type=str, default=[
                        '/home/macabaka/Workspace/ROV_four/datas/pid_parameters.json'], help='PID参数路径')
    parser.add_argument('--curve-path', nargs='+', type=str, default=[
                        '/home/macabaka/Workspace/ROV_four/datas/thrust_cureves.json'], help='推力曲线路径')
    parser.add_argument('--h750-path', nargs='+', type=str,
                        default=['/dev/ttyUSB1'], help='h750路径')
    parser.add_argument('--f407-path', nargs='+', type=str,
                        default=['/dev/ttyUSB0'], help='f407路径')
    opt = parser.parse_args()

    rclpy.init(args=args)  # 初始化rclpy
    node = CoreNode("uv_core", opt.h750_path[0],
                    opt.f407_path[0], opt.pid_path[0], opt.curve_path[0])  # 新建一个节点

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

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令
    rclpy.shutdown()  # 关闭rclpy
