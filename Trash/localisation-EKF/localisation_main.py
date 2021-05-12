#! /usr/bin/python3

from localisation_balloons import balloon_main
from navpy import lla2ned
import socket, sys
from _thread import *
from threading import Thread
import threading
import numpy as np
import csv
from datetime import datetime
import time
import os
import re
import math
from class_def import *
import GlobalVals

sys.path.insert(1,'../utils')
from navpy import lla2ned
from common import *


global buffer, gps_all, imu_all, gps_ref, positionXY, distance
buffer = GlobalVals.BUFFER


def position_update(posXY,new_gps):
    i = sysID_to_index(new_gps.sysID)
    GlobalVals.POSITIONXY[i-1,:]=posXY
    GlobalVals.GPS_ALL[i-1]=new_gps

def stringToGPS(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")

    except ValueError:
        
        return False, GPS()

    gps_i = GPS()

    try:
        gps_i.sysID = int(extract_string_data("'system': ",";",raw_data))
        gps_i.alt = float(extract_string_data("'altitude': ",";",raw_data))
        gps_i.lat = float(extract_string_data("'latitude': ",";",raw_data))
        gps_i.lon = float(extract_string_data("'longitude': ",";",raw_data))
        gps_i.epoch = float(extract_string_data("'time': ","}",raw_data))

        return True, gps_i

    except ValueError:

        return False, GPS()

def gps_callback(host,port):
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(GlobalVals.GPS_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the distanceEKF socket. This thread will now stop.")
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_LOGGER_THREAD = True
        return 


    while True:
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_LOGGER_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the distanceEKF socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        
        string_list = []
        iterator = data_str.find('{')

        while data_str.find('}', iterator) != -1:
            substring_end = data_str.find('}', iterator)
            string_list.append(data_str[iterator:substring_end + 1])
            iterator = substring_end + 1
        
        if len(string_list) > 0:
            gps_list = []
            for string in string_list:
                received, gps_i = stringToGPS(string)
                if received:
                    gps_list.append(gps_i)
            
            idx = 0
            while idx < len(gps_list):
                ned = lla2ned(gps_list[idx].lat, gps_list[idx].lon, gps_list[idx].atl, GlobalVals.GPS_REF.lat, GlobalVals.GPS_REF.lon, GlobalVals.GPS_REF.alt)
                posXY = ned[0:2]   
                position_update(posXY, gps_list[idx])
                idx += 1
    s.close()

def distanceEKF_callback(host,port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect((host,port))
        s.timeout(GlobalVals.DISTANCE_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the distanceEKF socket. This thread will now stop.")
        with GlobalVals.BREAK_DISTANCE_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_DISTANCE_LOGGER_THREAD = True
        return 
    
    synced = False

    while True:
        with GlobalVals.BREAK_DISTANCE_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_DISTANCE_LOGGER_THREAD:
                break

            try:
                data_bytes = s.recv(GlobalVals.BUFFER)
            except:
                print("Connection - distanceEKF - error")
                break

        if len(data_bytes) == 0:
            continue
        
        raw_data = data_bytes.decode('utf-8')
        
        if not synced:
            temp_index = raw_data.find("{")
            if temp_index == -1 or raw_data.find("}",temp_index) == -1:
                synced = False
                continue
            else:
                synced = True
                extract_string_data("{","}",raw_data)
        if synced:
            GlobalVals.DISTANCE = float(extract_string_data("DISTANCE:",";",raw_data)) 
            synced = False
    
    s.close()
    


if __name__ == "__main__":
    
    print("Localisation node started ...")

    GPS_Thread = Thread(target=gps_callback,args=(GlobalVals.HOST,GlobalVals.PORT_GPS))
    GPS_Thread.start()

    Distance_Thread = Thread(target=gps_callback,args=(GlobalVals.HOST,GlobalVals.PORT_DISTANCE))
    Distance_Thread.start()


    leader = 0                          
    sigma_range_measurement_val = 1     # this depends on the real data

    rateHz = 0.05                       # rate to run the localisation algorithm
    rate = 1/rateHz                     

    try:
        os.makedirs("datalog")
    except FileExistsError:
        pass

    file_name = "datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-localisationEKF.csv"
    
    # Logging
    location = GlobalVals.POSITIONXY

    time.sleep(1)

    print("Reading GPS signals ...")
    while True:
        if checkAllGPS(GlobalVals.GPS_ALL):
            break
    
    print("Algorithm started ....")

    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['p0x','p0y','p1x','p1y','p2x','p2y','p3x','p3y','p4x','p4y','l0x','l0y','l1x','l1y','l2x','l2y','l3x','l3y','l4x','l4y','iteration','execution_time','gps0_lon','gps0_lat','gps0_alt','gps1_lon','gps1_lat','gps1_alt','gps2_lon','gps2_lat','gps2_alt','gps3_lon','gps3_lat','gps3_alt','gps4_lon','gps4_lat','gps4_alt'])
    
        while True:
            
            positionXY_temp = GlobalVals.POSITIONXY
            gps_temp = GlobalVals.GPS_ALL
            distance = GlobalVals.DISTANCE


            start_time = time.time()
            location,_,iteration = balloon_main(leader,GlobalVals.ANCHOR_LIST,positionXY_temp,sigma_range_measurement_val,distance)
            execution_time = time.time()-start_time
            print('----- start printing ------')
            print("Balloon 1: lat", round(gps_temp[0,0],3), "lon: ", round(gps_temp[0,1],3))
            print("Balloon 2: lat", round(gps_temp[1,0],3), "lon: ", round(gps_temp[1,1],3))

            print("localisation error: \n", location-positionXY_temp)
            output.writerow([positionXY_temp[0,0],positionXY_temp[0,1],positionXY_temp[1,0],positionXY_temp[1,1],positionXY_temp[2,0],positionXY_temp[2,1],positionXY_temp[3,0],positionXY_temp[3,1],positionXY_temp[4,0],positionXY_temp[4,1],location[0,0],location[0,1],location[1,0],location[1,1],location[2,0],location[2,1],location[3,0],location[3,1],location[4,0],location[4,1],iteration,execution_time,gps_temp[0,0],gps_temp[0,1],gps_temp[0,2],gps_temp[1,0],gps_temp[1,1],gps_temp[1,2],gps_temp[2,0],gps_temp[2,1],gps_temp[2,2],gps_temp[3,0],gps_temp[3,1],gps_temp[3,2],gps_temp[4,0],gps_temp[4,1],gps_temp[4,2]])

            time.sleep(rate)

    if GPS_Thread.is_alive():
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_LOGGER_THREAD = True
        GPS_Thread.join()

    if Distance_Thread.is_alive():
        with GlobalVals.BREAK_DISTANCE_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_DISTANCE_LOGGER_THREAD = True
        Distance_Thread.join()
