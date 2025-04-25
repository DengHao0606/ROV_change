import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
import subprocess


class CaptureNode(Node):
    def __init__(self, name, front_cam, back_cam, host, height, width):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.size = (int(height), int(width))

        if front_cam != "none":
            # 前置摄像头
            self.front_cam_image_pub = self.create_publisher(
                Image, "front_cam_image/raw", 10)
            self.front_cam_timer = self.create_timer(
                0.02, self.front_cam_timer_callback)

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
            # 创建、管理子进程
            self.front_cam_pipe = subprocess.Popen(
                self.front_cam_command, stdin=subprocess.PIPE)
            self.front_cam_size = (int(self.front_cam_cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
                self.front_cam_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        if back_cam != "none":
            # 后置摄像头
            self.back_cam_image_pub = self.create_publisher(
                Image, "back_cam_image/raw", 10)
            self.back_cam_timer = self.create_timer(
                0.02, self.back_cam_timer_callback)

            self.back_cam_cap = cv2.VideoCapture(back_cam)

            self.back_cam_cap.set(6, cv2.VideoWriter_fourcc(
                'M', 'J', 'P', 'G'))  # 视频流格式
            self.back_cam_cap.set(5, 30)  # 帧率
            self.back_cam_cap.set(
                cv2.CAP_PROP_FRAME_WIDTH, int(width))     # 设置宽度
            self.back_cam_cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT, int(height))  # 设置长

            # 设置推流的参数
            self.back_cam_command = ['ffmpeg',
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
                                     "rtmp://"+host+"/cam/back"]
            # 创建、管理子进程
            self.back_cam_pipe = subprocess.Popen(
                self.back_cam_command, stdin=subprocess.PIPE)
            self.back_cam_size = (int(self.back_cam_cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
                self.back_cam_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def front_cam_timer_callback(self):

        # 以下三行为图像的消息转换，frame --> np.array --> imgmsg(可直接ros2发布)
        s, frame = self.front_cam_cap.read()

        if s:
            # self.get_logger().info('发布了图像')

            frame_compressed = cv2.resize(
                frame, self.size, interpolation=cv2.INTER_AREA)

            # 读取尺寸、推流
            img = cv2.resize(frame_compressed, self.front_cam_size)
            self.front_cam_pipe.stdin.write(img.tobytes())

        else:
            self.get_logger().info('图像获取失败')

    def back_cam_timer_callback(self):

        # 以下三行为图像的消息转换，frame --> np.array --> imgmsg(可直接ros2发布)
        s, frame = self.front_cam_cap.read()

        if s:
            # self.get_logger().info('发布了图像')

            frame_compressed = cv2.resize(
                frame, self.size, interpolation=cv2.INTER_AREA)

            # 读取尺寸、推流
            img = cv2.resize(frame_compressed, self.back_cam_size)
            self.back_cam_pipe.stdin.write(img.tobytes())

        else:
            self.get_logger().info('图像获取失败')


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-cam', nargs='+', type=str,
                        default=['none'], help='前置摄像头')
    parser.add_argument('--back-cam', nargs='+', type=str,
                        default=['none'], help='后置摄像头')
    parser.add_argument('--host', nargs='+', type=str,
                        default=["127.0.0.1"], help='srs服务器地址')
    parser.add_argument('--height', nargs='+', type=str,
                        default=["1280"], help='图像高度')
    parser.add_argument('--width', nargs='+', type=str,
                        default=["720"], help='图像宽度')
    opt = parser.parse_args()

    if opt.front_cam[0] == "none" and opt.back_cam[0] == "none":
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode(
            "uv_capture2web", opt.front_cam[0], opt.back_cam[0], opt.host[0], opt.height[0], opt.width[0])  # 新建一个节点

        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.spin(node)
        # 关闭rclpy
        rclpy.shutdown()
