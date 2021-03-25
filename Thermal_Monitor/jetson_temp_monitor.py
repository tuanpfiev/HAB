#!/usr/bin/env python3

import numpy as np
import thermal_zone
import os, sys
import time

sys.path.insert(1,'../utils')
from navpy import lla2ned
from common import *

def logTemperature(zone_paths, zone_names, file_name, sysID,time):

    zone_temps = thermal_zone.get_thermal_zone_temps(zone_paths)
    zone_temps = [t / 1000.0 for t in zone_temps]
    logString = list_to_str([sysID] + zone_temps + [time])
    try:
        fileObj = open(file_name, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Temperature Log: Error writting to file. Breaking thread.")
        print("Temperature Log: Exception: " + str(e.__class__))



if __name__ == "__main__":
    
    numArgs = len(sys.argv)
   
    if numArgs == 2:
        sysID = sys.argv[1]
    else:
        sysID = ""
    
    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-temperature.txt"
    GlobalVals.GPS_LOGGER_FILE = file_name

    
    
    zone_paths = thermal_zone.get_thermal_zone_paths()
    zone_names = thermal_zone.get_thermal_zone_names(zone_paths)
    
    logString = list_to_str(["sysID"] + zone_names + ["epoch"])

    try:
        fileObj = open(file_name, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Temperature Log: Error writting to file. Breaking thread.")
        print("Temperature Log: Exception: " + str(e.__class__))
      
    print('Start logging temperature ...')
    
    epoch = time.time()

    while True:
        logTemperature(zone_paths,zone_names, file_name, sysID,epoch)
        time.sleep(30)
