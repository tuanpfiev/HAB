#!/usr/bin/python
# -*- coding: utf-8 -*-
import os, sys
# add ../../Sources to the PYTHONPATH
# sys.path.append(os.path.join("..", "..", "Sources"))
sys.path.insert(1,'Sources')

import time
from threading import Thread
import GlobalVals

import socket

from yocto_api import *
from yocto_temperature import *


def die(msg):
    sys.exit(msg + ' (check USB cable)')

def main():
    errmsg = YRefParam()

    # Setup the API to use local USB devices
    if YAPI.RegisterHub("usb", errmsg) != YAPI.SUCCESS:
        sys.exit("init error" + errmsg.value)

    # retrieve any temperature sensor
    sensor = YTemperature.FirstTemperature()
    if sensor is None:
        die('No module connected')
    
    if not (sensor.isOnline()):
            die('device not connected')

    while sensor.isOnline():
       
        tempVal = sensor.get_currentValue()
        tempTime = time.time()
        with GlobalVals.appendTempDataMutex:
            GlobalVals.tempDataBuffer.append(tempVal)
            GlobalVals.timeDataBuffer.append(tempTime)

        with GlobalVals.newTempDataMutex:
            GlobalVals.newTempData = True
        
        logString = str(GlobalVals.sysID) + ','+ str(tempTime) + ',' + str(tempVal) + '\n'
        timeLocal = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(tempTime))

        print("SysID: ", GlobalVals.sysID,", Temperature: ",round(tempVal,1),", Time: ", timeLocal)
        try:
            fileObj = open(GlobalVals.fileName, "a")
            fileObj.write(logString)
            fileObj.close()
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error using error log file, ending error thread")
            break
        
        
        YAPI.Sleep(3000)

    YAPI.FreeAPI()

def threadTemperatureSocket():
    Logger_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Logger_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    Logger_Socket.bind((GlobalVals.HOST, GlobalVals.PORT_TEMPERATURE))
    Logger_Socket.settimeout(GlobalVals.socketTimeout)

    try: 
        Logger_Socket.listen(1)
        Logger_Connection, addr = Logger_Socket.accept()  
        Logger_Connection.settimeout(GlobalVals.socketTimeout) 
        print("Connected to: ",addr)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.endTempSocketMutex:
            GlobalVals.endTempSocket = True
        return 

    breakThread = False

    while not breakThread:
        # Check if socket ends
        with GlobalVals.endTempSocketMutex:
            if GlobalVals.endTempSocket:
                breakThread = True
                continue
        
        # Check if new data
        with GlobalVals.newTempDataMutex:
            newData = GlobalVals.newTempData
            GlobalVals.newTempData = False
        
        if newData:
            with GlobalVals.appendTempDataMutex:
                while len(GlobalVals.tempDataBuffer)>0:
                    tempVal = GlobalVals.tempDataBuffer.pop(0)
                    tempTime = GlobalVals.timeDataBuffer.pop(0)

                    socketPayload = "{'system': " + str(GlobalVals.sysID) + "; 'epoch': " + str(tempTime) + "; 'temp': " + str(tempVal) + ';}'
                    socketPayload = socketPayload.encode("utf-8")

                    try:
                        Logger_Connection.sendall(socketPayload)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error in the logger socket. Now closing thread.")
                        breakThread = True
                        break
        else:
            time.sleep(0.1)

    if breakThread:
        with GlobalVals.endTempSocketMutex:
            GlobalVals.endTempSocket = True

    Logger_Connection.close()

if __name__ == '__main__':
    numArgs = len(sys.argv)

    if numArgs >= 2:
        GlobalVals.sysID = sys.argv[1]
    print('sysID: ',GlobalVals.sysID)

    # create log file string 
    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    GlobalVals.fileName = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-TemperatureYoctopuce.txt"

    logString = "sysID, epoch, temperature\n"

    try:
        fileObj = open(GlobalVals.fileName, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error using error log file, ending error thread")

    temperatureThread = Thread(target=threadTemperatureSocket, args = ())
    temperatureThread.start()

    try: 
        main()
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")

    if temperatureThread.is_alive():
        with GlobalVals.endTempSocketMutex:
            GlobalVals.endTempSocket = True
        temperatureThread.join()

