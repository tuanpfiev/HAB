import random
import socket
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# server.bind(('127.0.0.1',5003))
addr = ('127.0.0.1',5003)
while True:
    # rand = random.randint(0,10)
    # msg, addr = server.recvfrom(1024)
    # msg = msg.upper()
    # print(addr)

    server.sendto(b'123',addr)
