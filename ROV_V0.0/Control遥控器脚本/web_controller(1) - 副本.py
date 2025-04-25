import json
import time
import socket
import keyboard
import argparse
import threading
import pygame

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
#   /   |     |  '.        S O N Y           .'  |  _  |   \
#  / ___| /|\ |___ \                        / ___| /_\ |___ \      (Y)
# / |      |      | ;                      ; | _         _ ||
# | | <---   ---> | |                      | ||_|       (_)||  (X)     (B)
# | |___   |   ___| ;                      ; |___       ___||
# |\    | \|/ |    /  _      ____      _   \    | (X) |    /|      (A)
# | \   |_____|  .','" "',  (_PS_)  ,'" "', '.  |_____|  .' |
# |  '-.______.-' /       \        /       \  '-._____.-'   |
# |               |  LJ   |--------|  RJ   |                |
# |              /\       /        \       /\               |
# |             /  '.___.'          '.___.'  \              |
# |            /                              \             |
#  \          /  x轴 axis(0)        x轴 axis(2) \           /   
#   \________/   y轴 axis(1)        y轴 axis(3)  \_________/   

#手柄键位分布 以及编号


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

def controller_curve(input):
    if input < 0 : 
        input = - input
        return - input**4
    return input**4

class Monitor():
    def __init__(self,mode):
        
        self.controller = CONTROLLER_INIT
        self.keyboard_state = KEYBOARD_STATE_INIT
        self.mode = mode
        self.stop = False

        keyboard.hook(lambda keyboard_event: self.keyboard_callback(keyboard_event))
        
    def keyboard_callback(self, keyboard_event):
        if keyboard_event.event_type == "up":
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

def joy_controller_callback(monitor):
    pygame.init()
    pygame.joystick.init()
    done=False

    while not done:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            if monitor.mode == 0:
                joystick_axis = joystick.get_axis(0),joystick.get_axis(1),joystick.get_axis(2),joystick.get_axis(3)
                
                if joystick_axis[3]**2 + joystick_axis[0]**2 >= 0.03 ** 2 :
                    monitor.controller["y"] = -2500.0 * controller_curve(joystick_axis[3])  # 前后
                    monitor.controller["x"] = 5000.0 * controller_curve(joystick_axis[0])  # 左右
                else :
                    monitor.controller["y"] = 0.0  # 前后
                    monitor.controller["x"] = 0.0  # 左右
                
                if joystick_axis[1]**2 + joystick_axis[2]**2 >= 0.03 ** 2 :
                    monitor.controller["z"] = -2500.0 * controller_curve(joystick_axis[1]) # 上下
                    monitor.controller["yaw"] = 250.0 * controller_curve(joystick_axis[2])  # 旋转
                else : 
                    monitor.controller["z"] = 0.0 # 上下
                    monitor.controller["yaw"] = 0.0  # 旋转
                if(joystick.get_axis(5) != -1):
                    monitor.controller["servo0"] = 0.49 * (joystick.get_axis(5) + 1 ) # 舵机
                    #monitor.controller["servo0"] = 0.10
                    if(joystick.get_axis(4) != -1):
                        ctr = 0.49 * (joystick.get_axis(5) + 1 )
                        monitor.controller["servo0"] = ctr
                elif(joystick.get_button(9)):
                    monitor.controller["servo0"] = 0.15
                elif(joystick.get_button(10)):
                    monitor.controller["servo0"] = ctr

if __name__ == "__main__":
    # 加载参数
    # host = "192.168.16.102"
    host = "192.168.16.102"
    port = "10086"
    mode = "0"


    '''host = input("主机地址?\r\n")
    port = input("主机端口?\r\n")
    mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
    '''
    monitor = Monitor(int(mode))
    server_address = (str(host), int(port))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 创建手柄监听线程
    thread_joy_controller = threading.Thread(target=joy_controller_callback,args=(monitor,))
    # 启动线程
    thread_joy_controller.start()


    while True:
        if monitor.stop == True:
            mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
            monitor.mode = int(mode)
            monitor.stop = False

        time.sleep(0.05)
        if monitor.mode == 1:
            monitor.controller_callback()
        print(monitor.controller)
        msg = json.dumps(monitor.controller) # 使用json传输数据
        client_socket.sendto(msg.encode(), server_address)

        
    