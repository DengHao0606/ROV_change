import socket
import json
 
PORT = 10087
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
address = (s.getsockname()[0], PORT)

print("service start at "+str(address))
server_socket.bind(address)
while True:
    receive_data, client_address = server_socket.recvfrom(1024)
    data =  json.loads(receive_data.decode())
    print(data)