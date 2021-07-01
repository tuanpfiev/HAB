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


from yocto_api import *
from yocto_tilt import *
from yocto_compass import *
from yocto_gyro import *
from yocto_accelerometer import *
from yocto_magnetometer import *


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
    qt = YQt.FindQt(serial + ".qt1")


    if not (tilt1.isOnline()):
        die("Module not connected (check identification and USB cable)")

    while tilt1.isOnline():
        ax = accelerometer.get_xValue()
        ay = accelerometer.get_yValue()
        az = accelerometer.get_zValue()
        # print(ax,ay,az)

        gx = gyro.get_xValue()
        gy = gyro.get_yValue()
        gz = gyro.get_zValue()
        # print(gx,gy,gz)

        magX = magneticVector.get_xValue()
        magY = magneticVector.get_yValue()
        magZ = magneticVector.get_zValue()

        qtW = qt.get_quaternionW()
        qtX = qt.get_quaternionX()
        qtY = qt.get_quaternionY()
        qtZ = qt.get_quaternionZ()

        roll = gyro.get_roll()
        pitch = gyro.get_pitch()
        yaw = gyro.get_heading()

        time_ms = time.time() * 1000

        msg = "{|SYSTEM_ID}: " + str(0) + "; " \
            + "EPOCH: " + str(time_ms) + "; " \
                + "ACCELERATION: " + str(ax) + "," + str(ay) + "," + str(az) + "; " \
                    + "GYRO: " + str(gx) + "," + str(gy) + "," + str(gz) + "; " \
                        + "MAGNETIC_VECTOR: " + str(magX) + "," + str(magY) + "," + str(magZ) + "; " \
                            + "RAW_QT: " + str(qtW) + "," + str(qtX) + "," + str(qtY) + "," + str(qtZ) + "; "\
                                + "EULER_321: " + str(roll) + "," + str(pitch) + "," + str(yaw) + ";}"
        
        
        
        
        YAPI.Sleep(100, errmsg)
    YAPI.FreeAPI()



if __name___ == '__main__':
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


    try: 
        main()
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")