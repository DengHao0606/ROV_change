"""
服务器测试程序
"""
import time
import logging
from uuv_webrtc import CvCapture
from uuv_webrtc import RtcServer

if __name__ == "__main__":
    cap = CvCapture(cam=0, frame_size=(1280, 720), fps=30)
    with RtcServer(cap=cap, port=20000, codec="video/VP8") as server:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            print("服务器通过Ctrl+C退出")