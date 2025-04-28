import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from sensor_msgs.msg import Image
import argparse
from uv_vision import stereocam


def image_publish(frame, img_bridge, image_pub):
    # 转为numpy.array
    frame = np.array(frame)
    # 转换为ros2消息类型，且解码方式为b(blue)、g(green)、r(red)
    data = img_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    # 发布 转换好的 图像类型消息
    image_pub.publish(data)


class CaptureNode(Node):
    def __init__(self, name, opt):
        #子类CaptureNode能够调用Node的属性
        super().__init__(name)
        #连接ros和opencv
        self.bridge = CvBridge()

        self.get_logger().info("大家好，我是%s!" % name)

        if opt.front_cam[0] != "none":
            # 前置摄像头
            self.frontcam_raw_pub = self.create_publisher(
                Image, "front_cam/raw", 10)
            
            self.frontcam_rectified_pub = self.create_publisher(
                Image, "front_cam/rectified", 10)

            self.front_cam_timer = self.create_timer(
                0.01, self.front_cam_timer_callback)

            #矫正
            self.front_sc = stereocam.StereoCamera()
            self.front_sc.device_parameter_config(device=opt.front_cam[0])
            if opt.front_params[0] != "none":
                self.front_sc.cal_parameters_init(opt.front_params[0])
                self.front_sc.rectification_init()

        self.opt =opt   


    def front_cam_timer_callback(self):

        s, f = self.front_sc.capture()

        if s:
            f = cv2.flip(f, 0)
            image_publish(f, self.bridge, self.frontcam_raw_pub)
            
            if self.opt.front_params[0] != "none":
                r_l = self.front_sc.rectifyImage(f)
                image_publish(r_l, self.bridge, self.frontcam_rectified_pub)


            self.get_logger().info('发布了图像')
        else:
            self.get_logger().info('图像获取失败')


def main(args=None):
    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-cam', nargs='+', type=str,
                        default=['/dev/video0'], help='前置摄像头')

    parser.add_argument('--front-params', nargs='+', type=str,
                        default=['/home/macabaka/robot.npz'], help='前置摄像头参数存储路径')

    opt = parser.parse_args()

    if opt.front_cam[0] == "none" and opt.back_cam[0] == "none":
        print("No Camera opened")
    else:
        rclpy.init(args=args)  # 初始化rclpy
        node = CaptureNode("uv_capture", opt)  # 新建一个节点
        rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
        rclpy.shutdown()  # 关闭rclpy
        
            
