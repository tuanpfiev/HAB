#! /usr/bin/python3

from localisation_balloons import balloon_main
from navpy import lla2ned
import socket, sys
from _thread import *
import threading
import numpy as np
import csv
from datetime import datetime
import time
import os
import re
import math

sys.path.insert(1,'../utils')
from navpy import lla2ned

global positionXY, buffer, gps_all
positionXY = np.array(([1.0,1.0],[2.0,2.0],[3.0,3.0],[4.0,4.0],[5.0,5.0]))
gps_all = np.array(([1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,3.0,3.0],[4.0,4.0,4.0],[5.0,5.0,5.0]))
buffer = 1024

global gps1_lon, gps1_lat, gps1_lat
gps1_lon = 0.0
gps1_lat = 0.0
gps1_alt = 0.0

global lon_ref, lat_ref, alt_ref
lon_ref = 142.1962
lat_ref = -36.7189
alt_ref = 0

def sysID_to_index(sysID: int):
    if sysID == 1:
        return 1
    elif sysID == 2:
        return 2
    elif sysID == 253:
        return 3
    elif sysID == 254:
        return 4
    elif sysID == 255:
        return 5
    else:
        print('SysID error')
        return 1


def position_update(sysID,posXY,lon,lat,alt):
    global positionXY, gps_all
    i = sysID_to_index(sysID)
    positionXY[i-1,:]=posXY
    gps_all[i-1,:]=[lon,lat,alt]


class stMercator:
    def __init__(self, System, Latitude, Longitude, Altitude, Epoch):

        self.Latitude = Latitude
        self.Longitude = Longitude
        self.Altitude = Altitude
        self.Epoch = Epoch
        self.System = System

def stringToMercator(raw_data):
    global positionXY
    try:

        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")

    except ValueError:
        
        return False, stMercator(0,0,0,0,0)

    t_Mercator = stMercator(0,0,0,0,0)

    try:

        system_begin_idx = raw_data.find("'system': ")
        system_end_idx = raw_data.find(';', system_begin_idx)
        t_Mercator.System = int(raw_data[system_begin_idx + 10:system_end_idx])

        altitude_begin_idx = raw_data.find("'altitude': ")
        altitude_end_idx = raw_data.find(';', altitude_begin_idx)
        t_Mercator.Altitude = float(raw_data[altitude_begin_idx + 12:altitude_end_idx])    

        latitude_begin_idx = raw_data.find("'latitude': ")
        latitude_end_idx = raw_data.find(';', latitude_begin_idx)
        t_Mercator.Latitude = float(raw_data[latitude_begin_idx + 12:latitude_end_idx])

        longitude_begin_idx = raw_data.find("'longitude': ")
        longitude_end_idx = raw_data.find(';', longitude_begin_idx)
        t_Mercator.Longitude = float(raw_data[longitude_begin_idx + 13:longitude_end_idx])

        time_begin_idx = raw_data.find("'time': ")
        time_end_idx = raw_data.find('}', time_begin_idx)
        t_Mercator.Epoch = float(raw_data[time_begin_idx + 8:time_end_idx])

        return True, t_Mercator

    except ValueError:

        return False, stMercator(0,0,0,0,0)


def position_callback(host,port):
    global positionXY, gps_all

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(2)
    except:
        pass

    while True:
        try:
            data_bytes = s.recv(buffer)
        except socket.timeout as e:
            err = e.args[0]
            if err == 'timed out':
                time.sleep(1)
                print('Receive time out, retrying...')
                continue
            else:
                print('1___'+str(e))
                os._exit(1)
        except socket.error as e:
            print(('2___'+str(e)))
            os._exit(1)
        else:
            if len(data_bytes) == 0:
                print('Blank msg!')
            else:
                data_str = data_bytes.decode('utf-8')

                string_list = []

                iterator = 0

                while data_str.find('}', iterator) != -1:

                    substring_end = data_str.find('}', iterator)

                    string_list.append(data_str[iterator:substring_end + 1])

                    iterator = substring_end + 1

                if len(string_list) > 0:

                    mercators = []

                    for string in string_list:

                        received, t_Mercator = stringToMercator(string)

                        if received:

                            mercators.append(t_Mercator)

                    idx = 0

                    while idx < len(mercators):
                        
                        ned = lla2ned(mercators[idx].Latitude, mercators[idx].Longitude, mercators[idx].Altitude, lat_ref, lon_ref, alt_ref)
                        posXY = ned[0:2]      
                        position_update(mercators[idx].System,posXY,mercators[idx].Longitude,mercators[idx].Latitude,mercators[idx].Altitude)

                        idx += 1



if __name__ == "__main__":
    
    host = '127.0.0.1'
    port = 5002

    start_new_thread(position_callback,(host,port))

    n_balloons = 5                      # No. of balloons
    leader = 0                          
    anchor_list = np.array([0,1,2])     # Balloons that are used as anchors
    sigma_range_measurement_val = 1     # this depends on the real data

    rateHz = 0.05                       # rate to run the localisation algorithm
    rate = 1/rateHz                     

    try:
        os.makedirs("datalog")
    except FileExistsError:
        pass

    file_name = "datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-datalog.csv"
    
    # Logging
    location = positionXY

    time.sleep(1)
    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['p0x','p0y','p1x','p1y','p2x','p2y','p3x','p3y','p4x','p4y','l0x','l0y','l1x','l1y','l2x','l2y','l3x','l3y','l4x','l4y','iteration','execution_time','gps0_lon','gps0_lat','gps0_alt','gps1_lon','gps1_lat','gps1_alt','gps2_lon','gps2_lat','gps2_alt','gps3_lon','gps3_lat','gps3_alt','gps4_lon','gps4_lat','gps4_alt'])
    
        while True:
            
            positionXY_temp = positionXY
            gps_temp = gps_all


            start_time = time.time()
            location,_,iteration = balloon_main(n_balloons,leader,anchor_list,positionXY_temp,sigma_range_measurement_val)
            execution_time = time.time()-start_time
            print('-----')
            print(location-positionXY_temp)
            output.writerow([positionXY_temp[0,0],positionXY_temp[0,1],positionXY_temp[1,0],positionXY_temp[1,1],positionXY_temp[2,0],positionXY_temp[2,1],positionXY_temp[3,0],positionXY_temp[3,1],positionXY_temp[4,0],positionXY_temp[4,1],location[0,0],location[0,1],location[1,0],location[1,1],location[2,0],location[2,1],location[3,0],location[3,1],location[4,0],location[4,1],iteration,execution_time,gps_temp[0,0],gps_temp[0,1],gps_temp[0,2],gps_temp[1,0],gps_temp[1,1],gps_temp[1,2],gps_temp[2,0],gps_temp[2,1],gps_temp[2,2],gps_temp[3,0],gps_temp[3,1],gps_temp[3,2],gps_temp[4,0],gps_temp[4,1],gps_temp[4,2]])

            time.sleep(rate)

