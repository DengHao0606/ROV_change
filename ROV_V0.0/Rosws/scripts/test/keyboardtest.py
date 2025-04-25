import curses
import os
import time
import sys

#初始化curses
screen=curses.initscr()
#设置不回显
curses.noecho()
#设置不需要按回车立即响应
curses.cbreak()
#开启键盘模式w
screen.keypad(1)

#阻塞模式读取0 非阻塞 1
screen.nodelay(1)  

char=screen.getch()
os.system("clear")

while(True):
    sys.stdout.write("aaa\r\n")
    sys.stdout.write(str(char)+"\r\n")
    time.sleep(0.05)
    os.system("clear")
    char=-1
    char=screen.getch()
    
    #根据得到的值进行操作
    #无值为-1  其他为keyCode