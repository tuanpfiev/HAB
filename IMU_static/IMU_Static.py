import time
import socket
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    epoch = time.time()
    send_data = "{|SYSTEM_ID: 1; EPOCH: " + str(epoch) + "; ACCELERATION: 0,0,9.81; GYRO: 0,0,0; MAGNETIC_VECTOR: 0,0,0; RAW_QT: 0,0,0,1; EULER_321: 0,0,0;}"
    print(send_data)
    data_encoded = send_data.encode('utf-8')

    s.sendto(data_encoded,('127.0.0.1',5003))
    time.sleep(0.01)
s.close()
