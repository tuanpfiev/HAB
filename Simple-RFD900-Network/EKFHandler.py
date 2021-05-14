import socket
import struct
import time
from threading import Thread
import threading
from _thread import *
import GlobalVals
import CustMes 
import NetworkManager

import sys
sys.path.insert(1,'../utils')
from common import *
from common_class import *

GPS_DistroThreadLock = threading.Lock()


def gps_update(new_data):
    GlobalVals.EKF_GPS_ALL = new_data
    

#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
def EKFGPSLoggerSocket():

    # set up socket
    while True:
        try:
            socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
            socket_logger.connect((GlobalVals.HOST, GlobalVals.EKF_GPS_LOGGER_SOCKET))
            socket_logger.settimeout(GlobalVals.EKF_GPS_LOGGER_SOCKET_TIMEOUT)
            
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to EKF....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the EKFGPSlogger socket. This thread will now stop.")
                with GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD_MUTEX:
                    GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD = True
                return 
        break
    print('Connected to EKF!!!')
    # intialize variables 
    bufferRead = 1024

    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_EKF_GPS_LOGGER_THREAD:
                break

        # read the socket 
        while True:
            try:
                data_bytes = socket_logger.recv(bufferRead)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("EKFGPSLoggerSocket() Receive timed out. Retrying...")
                    time.sleep(0.1)
                    continue
                else:
                    print("EKFGPSLoggerSocket(): Receive Connection error.")
                    break
                break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = extract_str_btw_curly_brackets(data_str)
        
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
            
            gps_i = GlobalVals.EKF_GPS_ALL
        
            GPSData = CustMes.MESSAGE_GPS()
            GPSData.Longitude = gps_i.lon
            GPSData.Latitude = gps_i.lat
            GPSData.Altitude = gps_i.alt
            GPSData.GPSTime = gps_i.epoch
            GPSData.SystemID = gps_i.sysID

            # add data to the gps buffer 
            # with GlobalVals.EKF_GPS_DATA_BUFFER_MUTEX:
        
            # set the flag for the data 
            # with GlobalVals.RECIEVED_EKF_GPS_LOCAL_DATA_MUTEX:
            #     GlobalVals.RECIEVED_EKF_GPS_LOCAL_DATA = True

            # send GPS data to other balloons 
            GPSPacket = CustMes.MESSAGE_FRAME()
            GPSPacket.SystemID = GlobalVals.SYSTEM_ID
            GPSPacket.MessageID = 5
            GPSPacket.TargetID = 0
            GPSPacket.Payload = GPSData.data_to_bytes()
            NetworkManager.sendPacket(GPSPacket)
            # print(GPSData)
            # print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 
