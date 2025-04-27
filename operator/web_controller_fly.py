import json
import sys
import time
import socket
import keyboard
import argparse
import threading
import webbrowser
import subprocess
import pygame   #与游戏手柄进行一个交互的pygame模块，下面有个joystick的函数用于获取一些操作轴的相关信息的

#在windows读取的手柄的信息通过socket通道进行传输

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

# 使用字典，记录按键操作


#=========================================================================

#         L1   axis(4)                              R1 axis(5)
#          L2   button(9)                          R2  button(10)
#       _=====_                                  _=====_
#      / _____ \                                / _____ \
#    +.-'_____'-.------------------------------.-'_____'-.+
#   /   |     |  '.                          .'  |  _  |   \
#  / ___| /|\ |___ \                        / ___| /_\ |___ \      (3)
# / |      |      | ;                      ; | _         _ ||
# | | <---   ---> | |                      | ||_|       (_)||  (2)     (1)
# | |___   |   ___| ;                      ; |___       ___||
# |\    | \|/ |    /  _      ____      _   \    | (X) |    /|   (button(0))
# | \   |_____|  .','" "',  (_PS_)  ,'" "', '.  |_____|  .' |
# |  '-.______.-' /       \        /       \  '-._____.-'   |
# |               |  LJ   |--------|  RJ   |                |
# |              /\       /        \       /\               |
# |             /  '.___.'          '.___.'  \              |
# |            /                              \             |
#  \          /  x轴 axis(3)       x轴 axis(0) \           /   
#   \________/   y轴 axis(2)        y轴 axis(1)  \_________/   

#手柄键位分布 以及编号
#备注：   axis(0)方向，控制左右的方向。     axis(3)方向，控制yaw的方向。    axis(2)方向，控制前后的方向。    axis(1)方向，控制上下的方向。



#控制器的值
CONTROLLER_INIT = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "servo0": 0.0,
    "servo1": 0.0,
    "state":0
}


#手柄的状态
KEYBOARD_STATE_INIT = {
    "w":0,
    "a":0,
    "s":0,
    "d":0,
    "q":0,
    "e":0,
    "i":0,
    "k":0,
    "j":0,
}

#手柄的控制曲线
def controller_curve(input):
    if input < 0 : 
        input = - input
        return - input**3
    return input**3


#通过键盘从而来实现运动控制的进行
class Monitor():
    def __init__(self,mode):
        
        self.controller = CONTROLLER_INIT    #控制器的值
        self.keyboard_state = KEYBOARD_STATE_INIT  #手柄现在的状态赋值到私有变量当中
        self.mode = mode  
        self.stop = False

        keyboard.hook(lambda keyboard_event: self.keyboard_callback(keyboard_event)) #我们利用 keyboard.hook实现回调函数的用法，自动执行这个回调函数的，这里还有了lamba匿名函数的用法
        
    def keyboard_callback(self, keyboard_event):
        if keyboard_event.event_type == "up":      #按键松开还是按下
            self.keyboard_state[keyboard_event.name] = 0
            if keyboard_event.name == "j":
                if self.controller["servo0"] == 0:
                    self.controller["servo0"] = 100.0
                else :
                    self.controller["servo0"] = 0.0    
            elif keyboard_event.name == "p":
                self.mode = 3
                time.sleep(0.04)
                self.stop = True
        elif keyboard_event.event_type == "down":
            self.keyboard_state[keyboard_event.name] = 1
            if keyboard_event.name == "1":
                self.controller["state"] = 0
            elif keyboard_event.name == "2":
                self.controller["state"] = 1
            elif keyboard_event.name == "3":
                self.controller["state"] = 2
            elif keyboard_event.name == "4":
                self.controller["state"] = 3
            
    def controller_callback(self):
        self.controller["y"] = 100.0 * (self.keyboard_state["w"] - self.keyboard_state["s"])
        self.controller["x"] = 100.0 * (self.keyboard_state["a"] - self.keyboard_state["d"])
        self.controller["z"] = 100.0 * (self.keyboard_state["k"] - self.keyboard_state["i"])
        self.controller["yaw"] = 100.0 * (self.keyboard_state["e"] - self.keyboard_state["q"])







#将类对象作为一个参数传入了的，这里的pygame模板抽象了手柄的一些相关的驱动
mild_mode = True
def joy_controller_callback(monitor):  
    pygame.init() #功能化初始化调用的一个设置,
    pygame.joystick.init()#初始化交互的函数
    done=False
    servo0=[0.99, 0.2]#舵机最大开启角度，最大关闭及角度
    global mild_mode

    while not done:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()  #临时设置某些组合键为被按下状态
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)#获取操作轴的一些相关信息
            joystick.init()
            if monitor.mode == 0:
                joystick_axis = joystick.get_axis(0),joystick.get_axis(1),joystick.get_axis(2),joystick.get_axis(3)
                
                if abs(joystick_axis[3]) >= 0.05 :
                    monitor.controller["yaw"] = (400 >> mild_mode) * controller_curve(joystick_axis[3]/0.7)  # 旋转
                else :
                    monitor.controller["yaw"] = 0.0  #monitor.controller["x"] = 0.0  # 左右

                if abs(joystick_axis[2])>= 0.05 :
                    monitor.controller["y"] = (6000 >> (mild_mode)) * controller_curve(joystick_axis[2]/0.7)  # 前后
                else :
                    monitor.controller["y"] = 0.0  # 前后

                if abs(joystick_axis[0]) >= 0.05 :
                    monitor.controller["x"] = (-3000 >> mild_mode) * controller_curve(joystick_axis[0]/0.7)  # 左右
                else : 
                    monitor.controller["x"] = 0.0  # 左右 

                if abs(joystick_axis[1]) >= 0.05 :
                    monitor.controller["z"] = (1500 >> mild_mode)* controller_curve(joystick_axis[1]/0.7) # 上下
                else : 
                    monitor.controller["z"] = 0.0 # 上下  


                if(joystick.get_button(4)):# C拨码器上
                    monitor.controller["servo0"] = servo0[1]#（关）
                elif(joystick.get_button(5)):#C拨码器下
                    monitor.controller["servo0"] = servo0[0]#（开）
                elif(joystick.get_axis(5) != -1):#右旋钮
                    monitor.controller["servo0"] = (servo0[0] - servo0[1]) * (1 - (joystick.get_axis(5) / 0.7 + 1) / 2) + servo0[1] # 舵机
                    pass

                
                if(joystick.get_button(0)):  # A拨码器上
                    mild_mode = True
                
                if(joystick.get_button(1)):  # A拨码器下
                    mild_mode = False


def open_powershell_and_run_ssh():
    command = 'ssh macabaka@192.168.16.104'
    subprocess.run(["powershell", "-Command", command])
                
if __name__ == "__main__":
    # 加载参数
    host = "192.168.16.104"         #输入主机地址
    local_address = "192.168.31.96" #输入本地地址
    port = "10086"
    mode = "0"
    '''host = input("主机地址?\r\n")
    port = input("主机端口?\r\n")
    mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
    '''
    monitor = Monitor(int(mode))
    server_address = (str(host), int(port))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    #socket通信，使得局域网或者有路由器交换机的网络结构能够进行通信
    
    localdata_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    print("连接到服务器: %s:%s" % server_address)
    
    # 创建手柄监听线程
    thread_joy_controller = threading.Thread(target=joy_controller_callback,args=(monitor,))
    
    # 启动线程
    thread_joy_controller.start()
    
    running=True


    while running:
        if monitor.stop == True:
            mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
            monitor.mode = int(mode)
            monitor.stop = False

        time.sleep(0.05)
        #time.sleep(2)
        msg = json.dumps(monitor.controller) # 使用json传输数据，这里是将python的字典格式转为json字符串的形式的操作
        client_socket.sendto(msg.encode(), server_address)
        localdata = monitor.controller.copy()
        localdata["mode"] = mild_mode
        msg = json.dumps(localdata)
        
        localdata_socket.sendto(msg.encode(), (local_address, 20002))
        if monitor.mode == 1:
            monitor.controller_callback()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
        
        
        