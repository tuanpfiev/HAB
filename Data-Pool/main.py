#!/usr/bin/env python3

import socket, time
from _thread import *


dataPoolSocket = socket.socket()
host = '127.0.0.1'
port_data_pool = 5010

buffer = 1024

try:
    dataPoolSocket.bind((host, port_data_pool))
except socket.error as e:
    print(str(e))
print('Data-Pool: Waitiing for a Connection..')

dataPoolSocket.listen(15)


port_GPS = 5002        
port_IMU = 5003

def threaded_client(connection,data):
    while True:
        connection.send(str.encode(data))
    connection.close()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_GPS:
    s_GPS.connect((host, port_GPS))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s_IMU:
        # s_IMU.connect((host, port_IMU))
        s_IMU.connect((host, port_IMU))
        
        while True:
            client, address = dataPoolSocket.accept()
            data_GPS = s_GPS.recv(buffer) # Receive GPS

            start_new_thread(threaded_client, (client,data_GPS))

            time.sleep(0.005)

        dataPoolSocket.close()
