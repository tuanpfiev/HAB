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
import GlobalVals

sys.path.insert(1,'../utils')
from navpy import lla2ned
from common import *
from common_class import *


global buffer, gps_all, imu_all, gps_ref, positionXY, distance
buffer = GlobalVals.BUFFER

def list_to_str(list_args):
    list_str = ""
    for i in range(len(list_args)):
        list_str = list_str + str(list_args[i]) + ","
    return list_str[:-1] + "\n"

def rssi_update(new_data):
    GlobalVals.RSSI = new_data
    # print(GlobalVals.RSSI.epoch)

def position_update(posXYZ,new_gps):
    i = sysID_to_index(new_gps.sysID)
    # GlobalVals.POSITIONXY[i-1,:]=
    GlobalVals.POS_XYZ[i-1]=posXYZ
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

def stringToRSSI(raw_data):
    try:
        raw_data.index("RSSI_filter:")
        raw_data.index("distance:")
        raw_data.index("time:")

    except ValueError:
        
        return False, RSSI()

    rssi_i = RSSI()

    try:
        temp = extract_string_data("RSSI_filter: ",";",raw_data)
        rssi_i.rssi_filtered = float(extract_string_data("RSSI_filter: ",";",raw_data))
        rssi_i.distance = float(extract_string_data("distance: ",";",raw_data))
        rssi_i.epoch = float(extract_string_data("time: ",";",raw_data))

        return True, rssi_i

    except ValueError:

        return False, RSSI()

def gps_callback(host,port):
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(GlobalVals.GPS_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the GPS socket. This thread will now stop.")
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
            print("There was an error starting the GPS socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        
        string_list = []
        string_list = extract_str_btw_curly_brackets(data_str)
        
        if len(string_list) > 0:
            gps_list = []
            for string in string_list:
                received, gps_i = stringToGPS(string)
                if received:
                    gps_list.append(gps_i)
            
            idx = 0
            while idx < len(gps_list):
                ned = lla2ned(gps_list[idx].lat, gps_list[idx].lon, gps_list[idx].alt, GlobalVals.GPS_REF.lat, GlobalVals.GPS_REF.lon, GlobalVals.GPS_REF.alt)
                posXYZ = POS_XYZ(ned[0],ned[1])   
                position_update(posXYZ, gps_list[idx])
                idx += 1
    s.close()

def distanceRSSI_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(GlobalVals.RSSI_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the RSSI socket. This thread will now stop.")
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_THREAD = True
        return 

    while True:
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            if GlobalVals.BREAK_RSSI_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.RSSI_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the RSSI receiver socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        # print('data RSSI: ',data_str)
        string_list = []
        string_list = extract_str_btw_curly_brackets(data_str)

        
        if len(string_list) > 0:
            rssi_list = []
            for string in string_list:
                received, rssi_i = stringToRSSI(string)
                if received:
                    rssi_list.append(rssi_i)
            
            idx = 0
            while idx < len(rssi_list):
                rssi_update(rssi_list[idx])
                idx += 1
    s.close()
    


if __name__ == "__main__":
    
    print("Localisation node started ...")

    GPS_Thread = Thread(target=gps_callback,args=(GlobalVals.HOST,GlobalVals.PORT_GPS))
    GPS_Thread.start()

    RSSIThread = Thread(target=distanceRSSI_callback, args = (GlobalVals.HOST, GlobalVals.PORT_RSSI))
    RSSIThread.start()


    leader = 0                          
    sigma_range_measurement_val = 200     # this depends on the real data

    rateHz = 0.05                       # rate to run the localisation algorithm
    rate = 1/rateHz                     

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-localisationRSSI.txt"

    logString = "px1, py1, px2, py2, px3, py3, px4, py4, lx2, ly2, lx3, ly3, lx4, ly4, iteration, execution time, epoch, distance, gps1lat, gps1lon, gps2lat, gps2lon, gps3lat, gps3lon, gps4lat, gps4lon \n"
    
    try:
        fileObj = open(file_name, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Localisation: Error writting to file. Breaking thread.")
        print("Localisation: Exception: " + str(e.__class__))


    
    # Logging
    location = GlobalVals.POS_XYZ

    time.sleep(1)

    print("Reading GPS signals ...")
    while True:
        if GlobalVals.RSSI.epoch != 0.0 and checkAllGPS(GlobalVals.GPS_ALL):
            break
    
    print("Algorithm started ....")

    # with open(file_name,'w') as file:
    #     output = csv.writer(file)
    #     output.writerow(['p0x','p0y','p1x','p1y','p2x','p2y','p3x','p3y','l0x','l0y','l1x','l1y','l2x','l2y','l3x','l3y','iteration','execution_time','epoch','gps0_lon','gps0_lat','gps0_alt','gps1_lon','gps1_lat','gps1_alt','gps2_lon','gps2_lat','gps2_alt','gps3_lon','gps3_lat','gps3_alt','gps4_lon','gps4_lat','gps4_alt'])
    
    while True:
        
        posXYZ_tmp = GlobalVals.POS_XYZ
        gps_tmp = GlobalVals.GPS_ALL
        distance = GlobalVals.RSSI.distance

        start_time = time.time()
        location,_,iteration = balloon_main(leader,GlobalVals.ANCHOR_LIST,posXYZ_tmp,sigma_range_measurement_val,distance)
        execution_time = time.time()-start_time
        
        pos_error = np.zeros([GlobalVals.N_BALLOON,2])
        for i in range(GlobalVals.N_BALLOON):
            pos_error[i,:] = [location[i,0]-posXYZ_tmp[i].x, location[i,1]-posXYZ_tmp[i].y]

        print('----- start printing ------')
        print('Time: ', start_time)
        print("Balloon 1: lat", round(gps_tmp[0].lat,4), "lon: ", round(gps_tmp[0].lon,4))
        print("Balloon 2: lat", round(gps_tmp[1].lat,4), "lon: ", round(gps_tmp[1].lon,4))

        print("localisation error: \n", pos_error)
        logString = list_to_str([posXYZ_tmp[0].x, posXYZ_tmp[0].y,posXYZ_tmp[1].x, posXYZ_tmp[1].y, posXYZ_tmp[2].x, posXYZ_tmp[2].y, posXYZ_tmp[3].x, posXYZ_tmp[3].y, location[0,0],location[0,1],location[1,0],location[1,1],location[2,0],location[2,1],location[3,0],location[3,1], iteration,execution_time, start_time, distance, gps_tmp[0].lat, gps_tmp[0].lon, gps_tmp[1].lat, gps_tmp[1].lon, gps_tmp[2].lat, gps_tmp[2].lon, gps_tmp[3].lat, gps_tmp[3].lon])
        
        # output.writerow([posXYZ_tmp[0].x, posXYZ_tmp[0].y,posXYZ_tmp[1].x, posXYZ_tmp[1].y, posXYZ_tmp[2].x, posXYZ_tmp[2].y, posXYZ_tmp[3].x, posXYZ_tmp[3].y, location[0,0],location[0,1],location[1,0],location[1,1],location[2,0],location[2,1],location[3,0],location[3,1], iteration,execution_time, start_time, gps_tmp[0].lat, gps_tmp[0].lon, gps_tmp[1].lat, gps_tmp[1].lon, gps_tmp[2].lat, gps_tmp[2].lon, gps_tmp[3].lat, gps_tmp[3].lon])
                            # write log string to file  
        try:
            fileObj = open(file_name, "a")
            fileObj.write(logString)
            fileObj.close()
        except Exception as e:
            print("LoRa Radio: Error writting to file. Breaking thread.")
            print("LoRa Radio: Exception: " + str(e.__class__))
            break
        time.sleep(rate)

    if GPS_Thread.is_alive():
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_LOGGER_THREAD = True
        GPS_Thread.join()

    if RSSIThread.is_alive():
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_THREAD = True
        RSSIThread.join()

