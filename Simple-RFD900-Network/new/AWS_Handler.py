import boto3
from datetime import datetime
import calendar
import random
import time
import json
import socket, sys
from datetime import datetime
import time
import os
import numpy as np
from _thread import *
import threading
import sys
sys.path.insert(1,'../utils')
from common import *
from common_class import *
import GlobalValsAWS

stream_name = 'RMITballoon_Data'
k_client = boto3.client('kinesis', region_name='ap-southeast-2')

global count_history, telemetry_all
count_history = 0
telemetry_all = np.array([GPS()]*4)

def gps_update(new_gps):
    i = sysID_to_index(new_gps.sysID)
    telemetry_all[i-1]=new_gps

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
        gps_i.epoch = float(extract_string_data("'time': ",";",raw_data))

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
        print("There was an error starting the GPS socket. This thread will now stop.")
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_THREAD = True
        return 

    while True:
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.GPS_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the GPS receiver socket. This thread will now stop.")
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
                gps_update(gps_list[idx])
                idx += 1

    s.close()

def lambda_handler(n,sys_time,data_all):
        global count_history
        
        telemetry_data = []

        for i in range(len(data_all)):

            each_balloon = {
                'sysID': str(i), 
                'timeStamp': str(sys_time), 
                'lat': str(data_all[i].lat + random.uniform(-0.1,0.1)),
                'lon': str(data_all[i].lon+ random.uniform(-0.1,0.1)),
                'alt': str(data_all[i].alt+ random.uniform(-20,20))
            }
            telemetry_data.append(each_balloon)

        count_history = count_history + 1
        if count_history % 5 == 1:
            for i in range(len(data_all)):
                latLon = {
                    'sysID_h': str(i),
                    'time_h': str(sys_time),
                    'lat_h': str(data_all[i].lat),
                    'lon_h': str(data_all[i].lon),
                    'alt_h': str(data_all[i].alt),

                }
                telemetry_data.append(latLon)
            

        print(telemetry_data)
        
        response = k_client.put_record(
                StreamName=stream_name,
                Data=json.dumps(telemetry_data),
                PartitionKey=str(random.randrange(10000))
        )
        

if __name__== '__main__':
        host = '127.0.0.1'
        port = 5002
        n_balloon = 4
        

        start_new_thread(position_callback,(host,port))
        
        while(True):
            time0 = int(time.time())
            lambda_handler(n_balloon,time0,telemetry_all)
            print('here')
            time.sleep(3)