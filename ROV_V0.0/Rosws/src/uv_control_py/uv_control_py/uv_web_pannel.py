import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
import subprocess
import socket
import json
import threading
import logging
import os
from uv_msgs.msg import CabinState
from uv_msgs.msg import PropellerThrust
from uv_msgs.msg import RobotAxis
from uv_msgs.msg import WorkState

#WebRTC相关组件
from uuv_webrtc import CvCapture
from uuv_webrtc import RtcServer

COLOR_RED = (0, 0, 255)


class CaptureNode(Node):
    def __init__(self, name, front_cam, host, height, width):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.size = (int(height), int(width))
        self.height = int(height)
        self.width = int(width)
        self.address = ("", 0)
        self.letterheight = int(16*self.height/720)
        self.letterzoom = 0.5*self.height/720

        self.cabin_state = CabinState()
        self.propeller_thrust = PropellerThrust()
        self.robot_position = RobotAxis()
        self.work_state = WorkState()
        self.robot_speed = RobotAxis()

        self.controller_address = ("0.0.0.0", 0)

        # 初始化WebRTC视频服务
        self._init_webrtc(0, 20000, 1280, 720)

    #cap = CvCapture(cam=0, frame_size=(1280, 720), fps=30, logger=server_logger)
    #with RtcServer(cap=cap, port=20000, codec="video/VP8", logger=server_logger) as server

    #初始化webrtc服务
    def _init_webrtc(self, cam, port, width ,height):

        # # 创建自定义logger
        # self.webrtc_logger = logging.getLogger("WebRTC_Server")
        # self.webrtc_logger.setLevel(logging.INFO)
        # self.file_handler = logging.FileHandler(
        #     filename=os.path.join(os.path.dirname(__file__), 'server.log'),
        #     mode='w',
        #     encoding='utf-8'
        #     )
        # self.file_handler.setFormatter(logging.Formatter('[%(levelname)s] %(asctime)s - %(message)s'))
        # self.webrtc_logger.addHandler(self.file_handler)

        # 创建视频捕获
        self.front_cam_cap = CvCapture(
            cam=cam,
            frame_size=(int(width), int(height)),
            fps=30
            # logger=self.webrtc_logger
        )
        # # 创建WebRTC服务器
        # self.webrtc_server = RtcServer(
        #     cap=self.front_cam_cap,
        #     port=int(port),
        #     codec="video/VP8"
        #     # logger=self.webrtc_logger
        # )
        # """
        # 启动WebRTC服务线程
        self.webrtc_thread = threading.Thread(
            target=self._run_webrtc_server, args=(port,),
            daemon=True
        )
        self.webrtc_thread.start()
        # """


    def _run_webrtc_server(self, port):
        """运行WebRTC服务器"""
        with RtcServer(cap=self.front_cam_cap, port=port, codec="video/VP8") as server:
            try:
                while rclpy.ok():
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            finally:
                self.get_logger().info("服务器通过Ctrl+C退出")


        """
        # 前置摄像头
        self.front_cam_cap = cv2.VideoCapture(front_cam)
        self.front_cam_cap.set(
            6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # 视频流格式
        self.front_cam_cap.set(5, 30)  # 帧率
        self.front_cam_cap.set(
            cv2.CAP_PROP_FRAME_WIDTH, int(width))     # 设置宽度
        self.front_cam_cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT, int(height))  # 设置长

        # 设置推流的参数
        self.front_cam_command = ['ffmpeg',
                                  '-y',
                                  '-f', 'rawvideo',
                                  '-vcodec', 'rawvideo',
                                  '-pix_fmt', 'bgr24',
                                  '-s', width+'*'+height,  # 根据输入视频尺寸填写
                                  '-r', '25',
                                  '-i', '-',
                                  '-c:v', 'h264',
                                  '-pix_fmt', 'yuv420p',
                                  '-preset', 'ultrafast',
                                  "-tune", "zerolatency",
                                  "-start_time_realtime", "0",
                                  '-f', 'flv',
                                  "rtmp://"+host+"/cam/front"]
        """


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
        
        """不再需要子进程推流
        # 创建、管理子进程
        self.front_cam_pipe = subprocess.Popen(
            self.front_cam_command, stdin=subprocess.PIPE)
        self.front_cam_size = (int(self.front_cam_cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
            self.front_cam_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        """


    def cabin_state_callback(self, data):
        self.cabin_state = data

    def propeller_thrust_callback(self, data):
        self.propeller_thrust = data

    def robot_position_callback(self, data):
        self.robot_position = data

    def robot_speed_callback(self, data):
        self.robot_speed = data

    #视频帧处理
    # def front_cam_timer_callback(self):
    #     s, frame = self.front_cam_cap.read()

    #     if s:
    #         buff = "Temp  : {:.1f}  Hum : {:.2f}  Leak : {:d}".format(
    #             self.cabin_state.temp, self.cabin_state.hum, self.cabin_state.leak)
    #         cv2.putText(frame, buff, (0, self.letterheight), cv2.FONT_HERSHEY_SIMPLEX,
    #                     self.letterzoom, COLOR_RED, 1, cv2.LINE_AA)

    #         buff = "Speed : {:.2f} Deepth : {:.2f}".format(
    #             self.robot_speed.y, self.robot_position.z)
    #         cv2.putText(frame, buff, (0, 2*self.letterheight), cv2.FONT_HERSHEY_SIMPLEX,
    #                     self.letterzoom, COLOR_RED, 1, cv2.LINE_AA)

    #         buff = "controller :  "+str(self.address)
    #         cv2.putText(frame, buff, (0, 3*self.letterheight), cv2.FONT_HERSHEY_SIMPLEX,
    #                     self.letterzoom, COLOR_RED, 1, cv2.LINE_AA)
            
    #         """
    #         frame_compressed = cv2.resize(
    #             frame, self.size, interpolation=cv2.INTER_AREA)

    #         # 读取尺寸、推流
    #         img = cv2.resize(frame_compressed, self.front_cam_size)
    #         self.front_cam_pipe.stdin.write(img.tobytes())
    #         """

    #     else:
    #         self.get_logger().info('图像获取失败')

    #     return frame

#未修改
def controller_callback(node, port):
    openloop_thrust = RobotAxis()
    servo_state = CabinState()
    work_state = WorkState()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    node.address = (s.getsockname()[0], port)

    server_socket.bind(node.address)

    while rclpy.ok():
        receive_data, client_address = server_socket.recvfrom(1024)
        data = json.loads(receive_data.decode())

        openloop_thrust.x = data["x"]
        openloop_thrust.y = data["y"]
        openloop_thrust.z = data["z"]
        openloop_thrust.yaw = data["yaw"]
        openloop_thrust.roll = data["roll"]
        openloop_thrust.pitch = data["pitch"]

        servo_state.servo[0] = data["servo0"]
        servo_state.servo[1] = data["servo1"]
        
        work_state.state = data["state"]

        node.openloop_thrust_pub.publish(openloop_thrust)
        node.servo_control_pub.publish(servo_state)
        node.work_state_pub.pubilsh(work_state)
        # node.get_logger().info(str(data))
        

# def webrtc_viewer(node):
#     while rclpy.ok():
#         node.front_cam_timer_callback()

"""#改为使用webrtc
def web_viewer(node):
    while rclpy.ok():
        node.front_cam_timer_callback()
"""

def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--cam', nargs='+', type=str,
                        default=['none'], help='前置摄像头')
    parser.add_argument('--host', nargs='+', type=str,
                        default=["127.0.0.1"], help='srs服务器地址')
    parser.add_argument('--height', nargs='+', type=str,
                        default=["1080"], help='图像高度')
    parser.add_argument('--width', nargs='+', type=str,
                        default=["1920"], help='图像宽度')
    parser.add_argument('--port', nargs='+', type=str,
                        default=["10086"], help='远程控制器接入端口')
    opt = parser.parse_args()

    if opt.cam[0] == "none":
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode(
            "uv_web_pannel", opt.cam[0], opt.host[0], opt.height[0], opt.width[0])  # 新建一个节点
        
        """
        thread_viewer = threading.Thread(
            target=web_viewer, args=(node,))
        thread_viewer.start()
        """


        # #启动视频线程
        # thread_viewer = threading.Thread(
        #     target=webrtc_viewer, args=(node,))
        # thread_viewer.start()
        #启动控制线程
        thread_controller = threading.Thread(
            target=controller_callback, args=(node, int(opt.port[0])))
        thread_controller.start()

        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.spin(node)
        # 关闭rclpy
        rclpy.shutdown()
