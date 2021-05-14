import socket
import struct
import time
from threading import Thread
import threading

import GlobalVals
import CustMes 
import NetworkManager

import sys
sys.path.insert(1,'../utils')
from common import *
from common_class import *


def temperatureUpdate(new_data):
    GlobalVals.TEMPERATURE_ALL = new_data


#=====================================================
# Thread for local IMU logger socket connection 
#=====================================================
def TemperatureLoggerSocket():

    try:
        socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        socket_logger.connect((GlobalVals.HOST, GlobalVals.TEMP_LOGGER_SOCKET))
        socket_logger.settimeout(GlobalVals.TEMP_LOGGER_SOCKET_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the TemperatureLoggerSocket logger socket. This thread will now stop.")
        with GlobalVals.BREAK_TEMP_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_TEMP_LOGGER_THREAD = True
        return 
    
    bufferRead = 1024
    while True:
        # print("TEMPERATURE SOCKET")
        # if flag is set break the thread 
        with GlobalVals.BREAK_TEMP_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_TEMP_LOGGER_THREAD:
                break

        # read the socket 
        while True:
            try:
                data_bytes = socket_logger.recv(bufferRead)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("temperature Socket receive timed out. Retrying...")
                    time.sleep(0.1)
                    continue
                else:
                    print("Temperature Socket Connection error.")
                    break
                break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        
        string_list = extract_str_btw_curly_brackets(data_str)
        
        if len(string_list) > 0:
            temperature_list = []

            for string in string_list:
                received, temp_i = stringToTemperature(string)
                if received:
                    temperature_list.append(temp_i)

            idx = 0
            while idx < len(temperature_list):
                temperatureUpdate(temperature_list[idx])
                idx += 1
            
            temp_i = GlobalVals.TEMPERATURE_ALL
        
            TempData = CustMes.MESSAGE_TEMP()
            TempData.Temperature = temp_i.temperature
            TempData.Epoch = temp_i.epoch

            TempData.SystemID = temp_i.sysID

            # add data to the gps buffer 
            # with GlobalVals.EKF_GPS_DATA_BUFFER_MUTEX:
        
            # set the flag for the data 
            # with GlobalVals.RECIEVED_EKF_GPS_LOCAL_DATA_MUTEX:
            #     GlobalVals.RECIEVED_EKF_GPS_LOCAL_DATA = True

            # send GPS data to other balloons 
            TempPacket = CustMes.MESSAGE_FRAME()
            TempPacket.SystemID = GlobalVals.SYSTEM_ID
            TempPacket.MessageID = 6
            TempPacket.TargetID = 0
            TempPacket.Payload = TempData.data_to_bytes()
            NetworkManager.sendPacket(TempPacket)
            # print(TempData)
            # print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 

