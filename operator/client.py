"""
客户端测试程序（带控制器数据接收线程）
"""
import cv2
import socket
import threading
import json
from uuv_webrtc import RtcClient
import tkinter as tk

# 全局变量和锁
controller_data = None
data_lock = threading.Lock()



def draw_status(frame):
    global controller_data
    # 确定模式颜色
    mode_color = (0, 255, 0) if controller_data['mode'] else (0, 0, 255)
    mode_text = f"{'Mild Mode' if controller_data['mode'] else 'Wild Mode'}"

    status_text = [
        mode_text,
        f"X: {controller_data['x']:.2f}",
        f"Y: {controller_data['y']:.2f}",
        f"Z: {controller_data['z']:.2f}",
        f"Yaw: {controller_data['yaw']:.2f}",
        f"Servo: {controller_data['servo0']:.2f}"
    ]
    
    y_offset = 36
    for i, text in enumerate(status_text):
        # 设置文字颜色（第一行用模式色，其他用白色）
        text_color = mode_color if i == 0 else (255, 255, 255)
        
        # 先绘制黑色轮廓
        cv2.putText(frame, text, (24, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 5, cv2.LINE_AA)
        # 再绘制主体文字
        cv2.putText(frame, text, (24, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, text_color, 2, cv2.LINE_AA)
        y_offset += 36

def udp_listener(port):
    """UDP数据接收线程"""
    global controller_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))
    print(f"启动UDP监听，端口: {port}")
    
    while True:
        try:
            # 接收数据（假设为JSON格式）
            data, addr = sock.recvfrom(1024)
            decoded_data = json.loads(data.decode())
            
            # 使用锁更新全局变量
            with data_lock:
                controller_data = decoded_data
                
        except Exception as e:
            print(f"数据接收错误: {str(e)}")

if __name__ == "__main__":
    # 配置参数
    UDP_PORT = 20002  # 新增的UDP数据端口
    
    with RtcClient(
        local_port=20001,
        server_address=("192.168.16.104", 20000)
    ) as client:
        # 创建可调整窗口
        cv2.namedWindow("ROV video", cv2.WINDOW_NORMAL)
        
        # 设置初始窗口尺寸为屏幕分辨率
        root = tk.Tk()
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy()
        # screen_width = 1920  # 改为你的实际屏幕宽度
        # screen_height = 1080 # 改为你的实际屏幕高度
        cv2.resizeWindow("ROV video", screen_width, screen_height)
        
        # 启动UDP监听线程
        udp_thread = threading.Thread(
            target=udp_listener, 
            args=(UDP_PORT,),
            daemon=True
        )
        udp_thread.start()

        try:
            while True:
                # 视频处理部分
                success, frame = client.getLatestFrame()
                if success:
                    draw_status(frame)
                    cv2.imshow("ROV video", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

        except KeyboardInterrupt:
            pass
        finally:
            print("客户端退出")
            cv2.destroyAllWindows()