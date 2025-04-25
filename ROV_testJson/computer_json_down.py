import serial
import json
import time

print("测试")

ser = serial.Serial('COM8', 115200, timeout=1)  # 修改为你的端口

def send_json(cmd, value, name, number):
    data = {"cmd": cmd, "value": value, "name": name, "number":number}
    ser.write((json.dumps(data) + '\n').encode())  # 发送 JSON + 换行符

try:
    while True:
        send_json("led", 1, "DengHao", 111)  # 开 LED
        time.sleep(1)
        send_json("led", 0, "ZhongWeiJia", 222)  # 关 LED
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()