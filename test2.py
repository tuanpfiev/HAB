
import socket, time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 5003        # The port used by the server

while True:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((HOST,PORT))
	
	while True:
		data, addr = sock.recvfrom(1024)
		print(data)
