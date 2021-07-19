# ********************************************************************
#
#  $Id: helloworld.py 32630 2018-10-10 14:11:07Z seb $
#
#  An example that show how to use a  Yocto-3D
#
#  You can find more information on our web site:
#   Yocto-3D documentation:
#      https://www.yoctopuce.com/EN/products/yocto-3d/doc.html
#   Python API Reference:
#      https://www.yoctopuce.com/EN/doc/reference/yoctolib-python-EN.html
#
# *********************************************************************

#!/usr/bin/python
# -*- coding: utf-8 -*-
import os, sys
sys.path.insert(1,'Sources')

import GlobalVals
import time
import math

from threading import Thread
import socket

from yocto_api import *
from yocto_tilt import *
from yocto_compass import *
from yocto_gyro import *
from yocto_accelerometer import *
from yocto_magnetometer import *

DEG2RAD = math.pi/180
GRAV_0 = 9.81
def usage():
    scriptname = os.path.basename(sys.argv[0])
    print("Usage:")
    print(scriptname + ' <serial_number>')
    print(scriptname + ' <logical_name>')
    print(scriptname + ' any  ')
    sys.exit()


def die(msg):
    sys.exit(msg + ' (check USB cable)')

def main():
    errmsg = YRefParam()

    target = 'any'

    # Setup the API to use local USB devices
    if YAPI.RegisterHub("usb", errmsg) != YAPI.SUCCESS:
        sys.exit("init error" + errmsg.value)

    if target == 'any':
        # retreive any tilt sensor
        anytilt = YTilt.FirstTilt()
        if anytilt is None:
            die('No module connected (check USB cable)')
        m = anytilt.get_module()
        target = m.get_serialNumber()
    else:
        anytilt = YTilt.FindTilt(target + ".tilt1")
        if not (anytilt.isOnline()):
            die('Module not connected (check identification and USB cable)')

    serial = anytilt.get_module().get_serialNumber()
    tilt1 = YTilt.FindTilt(serial + ".tilt1")
    # tilt2 = YTilt.FindTilt(serial + ".tilt2")
    # tilt3 = YTilt.

    compass = YCompass.FindCompass(serial + ".compass")
    accelerometer = YAccelerometer.FindAccelerometer(serial + ".accelerometer")
    gyro = YGyro.FindGyro(serial + ".gyro")
    magneticVector = YMagnetometer.FindMagnetometer(serial + ".magnetometer")
    qt = YGyro.FindGyro(serial + ".qt1")


    if not (tilt1.isOnline()):
        die("Module not connected (check identification and USB cable)")

    while tilt1.isOnline():
        time_start = time.time()
        ax = accelerometer.get_xValue() * GRAV_0
        ay = accelerometer.get_yValue() * GRAV_0
        az = accelerometer.get_zValue() * GRAV_0
        # print(ax,ay,az)

        gx = gyro.get_xValue() * DEG2RAD
        gy = gyro.get_yValue() * DEG2RAD
        gz = gyro.get_zValue() * DEG2RAD
        # print(gx,gy,gz)

        magX = magneticVector.get_xValue()
        magY = magneticVector.get_yValue()
        magZ = magneticVector.get_zValue()

        qtW = gyro.get_quaternionW()
        qtX = gyro.get_quaternionX()
        qtY = gyro.get_quaternionY()
        qtZ = gyro.get_quaternionZ()
        # print("qt: ",math.sqrt(qtW**2 + qtX**2 + qtY**2 + qtZ**2))
        qt_vec = math.sqrt(qtW**2 + qtX**2 + qtY**2 + qtZ**2)
        if qt_vec < 1e-5:
            qt_vec = 1e-5

        qtW = qtW/qt_vec
        qtX = qtX/qt_vec
        qtY = -qtY/qt_vec
        qtZ = -qtZ/qt_vec

        roll = gyro.get_roll()
        pitch = - gyro.get_pitch()
        yaw = gyro.get_heading()

        time_ms = round(time.time() * 1000,1)

        msg = "{|SYSTEM_ID}: " + str(0) + "; " \
            + "EPOCH: " + str(time_ms) + "; " \
                + "ACCELERATION: " + str(ax) + "," + str(ay) + "," + str(az) + "; " \
                    + "GYRO: " + str(gx) + "," + str(gy) + "," + str(gz) + "; " \
                        + "MAGNETIC_VECTOR: " + str(magX) + "," + str(magY) + "," + str(magZ) + "; " \
                            + "RAW_QT: " + str(qtW) + "," + str(qtX) + "," + str(qtY) + "," + str(qtZ) + "; "\
                                + "EULER_321: " + str(roll) + "," + str(pitch) + "," + str(yaw) + ";}"

        with GlobalVals.DATA_BUFFER_MUTEX:
            GlobalVals.DATA_BUFFER.append(msg)

        with GlobalVals.NEW_DATA_MUTEX:
            GlobalVals.NEW_DATA = True

        logString = str(GlobalVals.sysID) + ',' + str(time_ms) + ',' + str(ax) + ',' + str(ay) + ',' + str(az)\
            + ',' + str(gx) + ',' + str(gy) + ',' + str(gz) \
                + ',' + str(magX) + ',' + str(magY) + ',' + str(magZ) \
                    + ',' + str(qtW) + ',' + str(qtX) + ',' + str(qtY) + ',' + str(qtZ) \
                        + ',' + str(roll) + ',' + str(pitch) + ',' + str(yaw) + '\n'

        try:
            fileObj = open(GlobalVals.fileName, "a")
            fileObj.write(logString)
            fileObj.close()
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error using error log file, ending error thread")
            break

        # msg = "{|SYSTEM_ID}: " + str(0) + "; " \
        #     + "EPOCH: " + str(time_ms) + "; " \
        #         + "ACCELERATION: " + str(round(ax,3)) + "," + str(round(ay,3)) + "," + str(round(az,3)) + "; " \
        #             + "GYRO: " + str(round(gx,5)) + "," + str(round(gy,5)) + "," + str(round(gz,5)) + "; " \
        #                 + "MAGNETIC_VECTOR: " + str(magX) + "," + str(magY) + "," + str(magZ) + "; " \
        #                     + "RAW_QT: " + str(round(qtW,4)) + "," + str(round(qtX,4)) + "," + str(round(qtY,4)) + "," + str(round(qtZ,4)) + "; "\
        #                         + "EULER_321: " + str(roll) + "," + str(pitch) + "," + str(yaw) + ";}"
        # print(msg)
        
        time_end=time.time()
        YAPI.Sleep(500, errmsg)
        print('interval: ',time.time()-time_start ,'(',time_end-time_start,')')
    YAPI.FreeAPI()

def threadIMU_Socket():
    Logger_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Logger_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    Logger_Socket.bind((GlobalVals.host, GlobalVals.port))
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
        with GlobalVals.END_IMU_SOCKET_MUTEX:
            if GlobalVals.END_IMU_SOCKET:
                breakThread = True
                continue
        
        # Check if new data
        with GlobalVals.NEW_DATA_MUTEX:
            newData = GlobalVals.NEW_DATA
            GlobalVals.NEW_DATA = False
        
        if newData:
            with GlobalVals.DATA_BUFFER_MUTEX:
                while len(GlobalVals.DATA_BUFFER)>0:
                    msg = GlobalVals.DATA_BUFFER.pop(0)
                    socketPayload = msg.encode("utf-8")

                    try:
                        Logger_Connection.sendall(socketPayload)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error in the logger socket. Now closing thread.")
                        breakThread = True
                        break
        else:
            time.sleep(0.01)

    if breakThread:
        with GlobalVals.END_IMU_SOCKET_MUTEX:
            GlobalVals.END_IMU_SOCKET = True

    Logger_Connection.close()

if __name__ == '__main__':
    numArgs = len(sys.argv)
    if numArgs >= 2:
        GlobalVals.sysID = sys.argv[1]
    print('sysID: ',GlobalVals.sysID)

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    GlobalVals.fileName = "../datalog/" + time.strftime("%Y%m%d-%H%M%S")+"-IMU_Yoctopuce.txt"
    
    logString = "sysID, epoch, ax, ay, az, gx, gy, gz, magX, magY, magZ, qw, qx, qy, qz, roll, pitch, yaw"

    try:
        fileObj = open(GlobalVals.fileName,"a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error using error log file, ending ...")

    IMU_Thread = Thread(target=threadIMU_Socket, args = ())
    IMU_Thread.start()

    try: 
        main()
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")

    if IMU_Thread.is_alive():
        with GlobalVals.END_IMU_SOCKET_MUTEX:
            GlobalVals.END_IMU_SOCKET = True
        IMU_Thread.join()