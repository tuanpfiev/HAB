
import socket, time

# HOST = '127.0.0.1'  # The server's hostname or IP address
# PORT = 5003        # The port used by the server

# while True:
# 	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 	sock.bind((HOST,PORT))
	
# 	while True:
# 		data, addr = sock.recvfrom(1024)
# 		print(data)
try:
	socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
	socket_logger.connect(('127.0.0.1',6000))
	socket_logger.settimeout(GlobalVals.EKF_GPS_LOGGER_SOCKET_TIMEOUT)
except Exception as e:
	print("Exception: " + str(e.__class__))
	print("There was an error starting the logger socket. This thread will now stop.")
	# with GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD_MUTEX:
	#     GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD = True
	# return 