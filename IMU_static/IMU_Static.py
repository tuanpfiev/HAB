import time
import socket
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    epoch = time.time()
    send_data = "{|SYSTEM_ID: 1; EPOCH: " + str(epoch*1000) + "; ACCELERATION: -0.582528,0.135325,9.798309; GYRO: 0.010906,0.008090,0.017289; MAGNETIC_VECTOR: -0.060494,0.056964,0.076564; RAW_QT: 0.689084,-0.014562,0.023355,0.724159; EULER_321: 0.789316,3.053983,92.864426;}"
    # send_data = "{|SYSTEM_ID: 1; EPOCH: " + str(epoch*1000) + "; ACCELERATION: 0,0,9.81; GYRO: 0,0,0; MAGNETIC_VECTOR: 0,0,0; RAW_QT: 0,0,0,1; EULER_321: 0,0,0;}"
    print(send_data)
    data_encoded = send_data.encode('utf-8')

    s.sendto(data_encoded,('127.0.0.1',5003))
    time.sleep(0.01)
s.close()
