import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse


class CaptureNode(Node):
    def __init__(self, name, front_cam, back_cam):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

        self.img_bridge = CvBridge()


        if front_cam != "none" :
            # 前置摄像头
            self.front_cam_image_pub = self.create_publisher(Image, "front_cam_image/raw", 10)
            self.front_cam_timer = self.create_timer(0.02, self.front_cam_timer_callback)

            self.front_cam_cap = cv2.VideoCapture(front_cam)

            self.front_cam_cap.set(6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # 视频流格式
            self.front_cam_cap.set(5, 30)  # 帧率
            self.front_cam_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)     # 设置宽度
            self.front_cam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 设置长

        if back_cam != "none" :
            # 后置摄像头
            self.back_cam_image_pub = self.create_publisher(Image, "back_cam_image/raw", 10)
            self.back_cam_timer = self.create_timer(0.02, self.back_cam_timer_callback)

            self.back_cam_cap = cv2.VideoCapture(back_cam)

            self.back_cam_cap.set(6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # 视频流格式
            self.back_cam_cap.set(5, 30)  # 帧率
            self.back_cam_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)     # 设置宽度
            self.back_cam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 设置长

    def front_cam_timer_callback(self):

        # 以下三行为图像的消息转换，frame --> np.array --> imgmsg(可直接ros2发布)
        s, frame = self.front_cam_cap.read()

        if s:
            self.get_logger().info('发布了图像')

            dim = (int(frame.shape[1] / 1), int(frame.shape[0] / 1))

            frame_compressed = cv2.resize(
                frame, dim, interpolation=cv2.INTER_AREA)

            frame = np.array(cv2.flip(frame_compressed,1))   # 镜像操作,且转为numpy.array

            # 转换为ros2消息类型，且解码方式为b(blue)、g(green)、r(red)
            data = self.img_bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            self.front_cam_image_pub.publish(data)  # 发布 转换好的 图像类型消息
        else:
            self.get_logger().info('图像获取失败')


    def back_cam_timer_callback(self):

        # 以下三行为图像的消息转换，frame --> np.array --> imgmsg(可直接ros2发布)
        s, frame = self.front_cam_cap.read()

        if s:
            self.get_logger().info('发布了图像')

            dim = (int(frame.shape[1] / 1), int(frame.shape[0] / 1))

            frame_compressed = cv2.resize(
                frame, dim, interpolation=cv2.INTER_AREA)

            frame = np.array(cv2.flip(frame_compressed,1))   # 镜像操作,且转为numpy.array

            # 转换为ros2消息类型，且解码方式为b(blue)、g(green)、r(red)
            data = self.img_bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            self.back_cam_image_pub.publish(data)  # 发布 转换好的 图像类型消息
        else:
            self.get_logger().info('图像获取失败')


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-cam', nargs='+', type=str,
                        default=['none'], help='前置摄像头')
    parser.add_argument('--back-cam', nargs='+', type=str,
                        default=['none'], help='后置摄像头')
    opt = parser.parse_args()

    if opt.front_cam[0] == "none" and opt.back_cam[0] == "none":
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode(
            "uv_capture", opt.front_cam[0], opt.back_cam[0])  # 新建一个节点
        rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.shutdown()  # 关闭rclpy
