import socket
import GlobalVals
import time

socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
socket_logger.connect((GlobalVals.HOST, GlobalVals.GPS_DISTRO_SOCKET))
socket_logger.settimeout(2)

bufferRead = 1024

while True:

    # read the socket 
    try:
        data_bytes = socket_logger.recv(bufferRead)
    except:
        print("Connection error.")
        break
    
    data_str = data_bytes.decode('utf-8')
    print(data_str)

    time.sleep(0.5)

socket_logger.close()
