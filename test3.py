#!/usr/bin/env python3

import socket, time


ServerSocket = socket.socket()
host = '127.0.0.1'
port = 5111
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print('Waitiing for a Connection..')
ServerSocket.listen(15)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((host, port))

    while True:
        Client, address = ServerSocket.accept()

        data = s.recv(1024)
        # Client.send(data)
        print(data.decode('utf-8'))


        time.sleep(1)
    # ServerSocket.close()
