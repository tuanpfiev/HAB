
import socket,time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 5002        # The port used by the server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:

        data = s.recv(2048)
        print('======================================')
        print('Received', (data))
        time.sleep(1)
