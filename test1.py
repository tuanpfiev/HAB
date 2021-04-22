import serial
import time

dataPort = serial.Serial('/dev/ttyTHS1',9600,timeout=1)

while True:
    try:
        data = dataPort.readline()
        if data:
            print(data)
            print('=====================================')
    except:
        dataPort.close()