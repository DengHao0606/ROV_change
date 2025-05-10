import socket
import time
import json

# 配置参数
local_ip = ''               # 本地IP（空表示所有接口）
local_port = 60000          # 本地端口
remote_ip = '192.168.0.10'  # 目标IP
remote_port = 5000          # 目标端口
interval = 1                # 发送间隔(秒)

# 创建JSON格式消息
message = {
    "timestamp": time.time(),
    "device": "EV",
    "status": "active",
    "data": {
        "voltage": 220.5,
        "current": 15.3,
        "temperature": 42.7
    }
}

# 创建并绑定Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((local_ip, local_port))

try:
    count = 0
    while True:
        # 将字典转换为JSON字符串并发送
        json_message = json.dumps(message)
        sock.sendto((json_message + '\r\n').encode(), (remote_ip, remote_port))
        count += 1
        print(f"Sent #{count} to {remote_ip}:{remote_port} - {json_message}")
        
        # 更新数据（示例：每次发送时修改部分数据）
        message["timestamp"] = time.time()
        message["data"]["voltage"] += 0.1
        message["data"]["current"] -= 0.05
        
        time.sleep(interval)
except KeyboardInterrupt:
    print("\nStopped")
finally:
    sock.close()