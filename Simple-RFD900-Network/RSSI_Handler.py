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
import copy



def gps_update(new_data):
    GlobalVals.EKF_GPS_ALL = new_data

def rssi_update(new_data):
    GlobalVals.RSSI_ALL = new_data
    

#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
def RSSI_LoggerSocket():

    # set up socket
    socket_logger = [None]* len(GlobalVals.RSSI_LOGGER_SOCKET)

    while True:
        for i in range(len(GlobalVals.RSSI_LOGGER_SOCKET))
        try:
            socket_logger[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
            socket_logger[i].connect((GlobalVals.HOST, GlobalVals.RSSI_LOGGER_SOCKET[i]))
            socket_logger[i].settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT)
            
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to RSSI....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the RSSI Logger socket. This thread will now stop.")
                with GlobalVals.BREAK_RSSI_LOGGER_THREAD_MUTEX:
                    GlobalVals.BREAK_RSSI_LOGGER_THREAD = True
                return 
        break
    print('Connected to RSSI!!!')
    # intialize variables 
    bufferRead = 1024

    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_RSSI_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_RSSI_LOGGER_THREAD:
                break

        # read the socket 
        for i in range(len(GlobalVals.RSSI_LOGGER_SOCKET)):



        try:
            data_bytes = socket_logger.recv(bufferRead)
        except:
            print("Connection error.")
            break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = extract_str_btw_curly_brackets(data_str):
        
        if len(string_list) > 0:
            rssi_list = []
            for string in string_list:
                received, rssi_i = stringToRSSI(string)
                if received:
                    rssi_list.append(rssi_i)

            idx = 0
            with GlobalVals.RSSI_UPDATE_MUTEX:
                while idx < len(rssi_list):
                    rssi_update(rssi_list[idx])
                    idx += 1
            
            with GlobalVals.RSSI_UPDATE_MUTEX:
                rssi_i = copy.deepcopy(GlobalVals.RSSI_ALL)
        
            RSSI_Data = CustMes.MESSAGE_RSSI()()
            RSSI_Data.FilteredRSSI = rssi_i.rssi_filtered
            RSSI_Data.Distance = rssi_i.distance
            RSSI_Data.Epoch = rssi_i.epoch

            # add data to the gps buffer 
            with GlobalVals.RSSI_DATA_BUFFER_MUTEX:
                GlobalVals.RSSI_DATA_BUFFER.append(RSSI_Data)

            # set the flag for the data 
            with GlobalVals.RECIEVED_RSSI_LOCAL_DATA_MUTEX: # 2 nodes?
                GlobalVals.RECIEVED_RSSI_LOCAL_DATA = True

            # send GPS data to other balloons 
            RSSI_Packet = CustMes.MESSAGE_FRAME()
            RSSI_Packet.SystemID = GlobalVals.SYSTEM_ID
            RSSI_Packet.MessageID = 7
            RSSI_Packet.TargetID = 0
            RSSI_Packet.Payload = RSSI_Data.data_to_bytes()
            NetworkManager.sendPacket(RSSI_Packet)
            print(RSSI_Packet)
            print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 
