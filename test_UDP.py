#!/usr/bin/env python3

# import socket, time

# HOST = '127.0.0.1'  # The server's hostname or IP address
# PORT = 5003        # The port used by the server

# with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
#     s.connect((HOST, PORT))

#     while True:
#         print('====')
#         data, server = s.recvfrom(1024)
#         print("---")
#         print(data.decode('utf-8'))

#         time.sleep(1)
import socket, time

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.settimeout(10)
client.bind(('127.0.0.1',5003))

while True:
    try:
        print('start')
        data, server = client.recvfrom(1024)
        print('check')
        print(data)
    except socket.timeout:
        print("time out")