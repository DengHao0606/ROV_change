# 测试摄像头
import cv2

if __name__ == '__main__':
    num = 1
    cap = cv2.VideoCapture("/dev/video0")
    cap.set(6, cv2.VideoWriter_fourcc('M','J','P','G'))# 视频流格式
    cap.set(5, 30)# 帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)     # 设置宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)  # 设置长度

    while True:
        s, f = cap.read()
        if s:
            cv2.imshow('src', f)

            if cv2.waitKey(10) & 0xff == ord('q'):
                break
    cv2.destroyAllWindows()