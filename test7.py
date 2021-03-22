
import socket,time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 5012        # The port used by the server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:

        data = s.recv(1024)
        print('======================================')
        print('Received', (data))
        time.sleep(1)
