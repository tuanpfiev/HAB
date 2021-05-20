import time
import socket
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    send_data = "a"
    data_encoded = send_data.encode('utf-8')

    s.sendto(data_encoded,('127.0.0.1',5003))
    time.sleep(0.01)
s.close()
