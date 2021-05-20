import socket
import struct
import time
from threading import Thread
import threading
from _thread import *
import GlobalVals
import CustMes 
import NetworkManager
import copy

import sys
sys.path.insert(1,'../utils')
from common import *
from common_class import *



def ekf_update(new_data):
    GlobalVals.EKF_ALL = new_data
    

#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
# def EKFLoggerSocket():

#     # set up socket
#     while True:
#         try:
#             socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
#             socket_logger.connect((GlobalVals.HOST, GlobalVals.EKF_LOGGER_SOCKET))
#             socket_logger.settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT)
            
#         except Exception as e:
#             if e.args[1] == 'Connection refused':
#                 print('Retry connecting to EKF....')
#                 time.sleep(1)
#                 continue
#             else:
#                 print("Exception: " + str(e.__class__))
#                 print("There was an error starting the EKFlogger socket. This thread will now stop.")
#                 with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
#                     GlobalVals.BREAK_EKF_LOGGER_THREAD = True
#                 return 
#         break
#     print('Connected to EKF!!!')
#     # intialize variables 
#     bufferRead = 1024
#     breakMainThread = False
#     while True:
        
#         # if flag is set break the thread 
#         with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
#             if GlobalVals.BREAK_EKF_LOGGER_THREAD:
#                 break

#         # read the socket 
#         while True:
#             try:
#                 data_bytes = socket_logger.recv(bufferRead)
#                 break
#             except Exception as e:
#                 if e.args[0] == 'timed out':
#                     print("EKFLoggerSocket() Receive timed out. Retrying...")
#                     time.sleep(0.1)
#                     continue
#                 else:
#                     print("EKFLoggerSocket(): Receive Connection error.")
#                     breakMainThread = True
#                     break
                
#         if breakMainThread:
#             break
        
#         # if there is nothing in the socket then it has timed out 
#         if len(data_bytes) == 0:
#             continue

#         data_str = data_bytes.decode('utf-8')
        
#         string_list = extract_str_btw_curly_brackets(data_str)
        
#         if len(string_list) > 0:
#             gps_list = []

#             for string in string_list:
#                 received, gps_i = stringToGPS(string)
#                 if received:
#                     gps_list.append(gps_i)

#             idx = 0
#             while idx < len(gps_list):
#                 gps_update(gps_list[idx])
#                 idx += 1
            
#             gps_i = GlobalVals.EKF_ALL
        
#             GPSData = CustMes.MESSAGE_GPS()
#             GPSData.Longitude = gps_i.lon
#             GPSData.Latitude = gps_i.lat
#             GPSData.Altitude = gps_i.alt
#             GPSData.GPSTime = gps_i.epoch
#             GPSData.SystemID = gps_i.sysID

#             # add data to the gps buffer 
#             with GlobalVals.EKF_DATA_BUFFER_MUTEX:
#                 GlobalVals.EKF_DATA_BUFFER.append(GPSData)

#             # set the flag for the data 
#             with GlobalVals.RECIEVED_EKF_LOCAL_DATA_MUTEX:
#                 GlobalVals.RECIEVED_EKF_LOCAL_DATA = True

#             # send GPS data to other balloons 
#             GPSPacket = CustMes.MESSAGE_FRAME()
#             GPSPacket.SystemID = GlobalVals.SYSTEM_ID
#             GPSPacket.MessageID = 5
#             GPSPacket.TargetID = 0
#             GPSPacket.Payload = GPSData.data_to_bytes()
#             NetworkManager.sendPacket(GPSPacket)
#             # print(GPSData)
#             # print("***************************")

#         # pause a little bit so the mutexes are not getting called all the time 
#         time.sleep(1)  

#     socket_logger.close()
#     return 

def EKFLoggerSocket():

    # set up socket
    while True:
        try:
            socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
            socket_logger.connect((GlobalVals.HOST, GlobalVals.EKF_LOGGER_SOCKET))
            socket_logger.settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT)
            
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to EKF....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the EKFlogger socket. This thread will now stop.")
                with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
                    GlobalVals.BREAK_EKF_LOGGER_THREAD = True
                return 
        break
    print('Connected to EKF!!!')
    # intialize variables 
    bufferRead = 1024
    breakMainThread = False
    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_EKF_LOGGER_THREAD:
                break

        # read the socket 
        while True:
            try:
                data_bytes = socket_logger.recv(bufferRead)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("EKFLoggerSocket() Receive timed out. Retrying...")
                    time.sleep(0.1)
                    continue
                else:
                    print("EKFLoggerSocket(): Receive Connection error.")
                    breakMainThread = True
                    break
                
        if breakMainThread:
            break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = extract_str_btw_curly_brackets(data_str)
        
        if len(string_list) > 0:
            ekf_list = []

            for string in string_list:
                received, ekf_i = stringToEKF(string)
                if received:
                    ekf_list.append(ekf_i)

            idx = 0
            with GlobalVals.EKF_UPDATE_MUTEX:
                while idx < len(ekf_list):
                    ekf_update(ekf_list[idx])
                    idx += 1
            
            ekf = copy.deepcopy(GlobalVals.EKF_ALL)
        
            EKF_Data = CustMes.MESSAGE_GPS()
            EKF_Data.Longitude = ekf.lon
            EKF_Data.Latitude = ekf.lat
            EKF_Data.Altitude = ekf.alt
            EKF_Data.GPSTime = ekf.epoch
            EKF_Data.SystemID = ekf.sysID
            EKF_Data.PosX = ekf.posX
            EKF_Data.PosY = ekf.posY
            EKF_Data.P00 = ekf.p00
            EKF_Data.P01 = ekf.p01
            EKF_Data.P10 = ekf.p10
            EKF_Data.P11 = ekf.p11

            # add data to the gps buffer 
            with GlobalVals.EKF_DATA_BUFFER_MUTEX:
                GlobalVals.EKF_DATA_BUFFER.append(EKF_Data)

            # set the flag for the data 
            with GlobalVals.RECIEVED_EKF_LOCAL_DATA_MUTEX:
                GlobalVals.RECIEVED_EKF_LOCAL_DATA = True

            # send GPS data to other balloons 
            EKF_Packet = CustMes.MESSAGE_FRAME()
            EKF_Packet.SystemID = GlobalVals.SYSTEM_ID
            EKF_Packet.MessageID = 5
            EKF_Packet.TargetID = 0
            EKF_Packet.Payload = EKF_Data.data_to_bytes()
            NetworkManager.sendPacket(EKF_Packet)
            # print(EKF_Data)
            # print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 


def EKF_AllDistributor():

    Distro_Socket = [None]*GlobalVals.N_EKF_NODE_PUBLISH
    Distro_Connection = [None]*GlobalVals.N_EKF_NODE_PUBLISH
    for i in range(GlobalVals.N_EKF_NODE_PUBLISH):
        # start socket 

        Distro_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Distro_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
        Distro_Socket[i].bind((GlobalVals.HOST, GlobalVals.EKF_ALL_DISTRO_SOCKET[i]))
        Distro_Socket[i].settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT)
        

        # Wait for connection on the distro socket 
        try:
            Distro_Socket[i].listen(1) 
            Distro_Connection[i], addr = Distro_Socket[i].accept()  
            Distro_Connection[i].settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT) 
            print("EKF All Distributor[",i,"] Connected to ", addr)                                            
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error in the EKF_AllDistributor[",i,"] socket. Now closing thread.")
            with GlobalVals.BREAK_EKF_ALL_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_EKF_ALL_DISTRO_THREAD = True
            return 0
  
    
    source1 = False
    source2 = False
    breakThread = False

    while True:

        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.BREAK_EKF_ALL_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_EKF_ALL_DISTRO_THREAD:
                break

        # check if local GPS data has been recived 
        with GlobalVals.RECIEVED_EKF_LOCAL_DATA_MUTEX:
            if GlobalVals.RECIEVED_EKF_LOCAL_DATA:
                source1 = True
                GlobalVals.RECIEVED_EKF_LOCAL_DATA_MUTEX = False
        
        # check if GPS data from the radio has been recieved 
        with GlobalVals.RECIEVED_EKF_RADIO_DATA_MUTEX:
            if GlobalVals.RECIEVED_EKF_RADIO_DATA:
                source2 = True
                GlobalVals.RECIEVED_EKF_RADIO_DATA = False
        
        # if no data has been recieved sleep and loop
        if not source1 and not source2:
            time.sleep(0.1)
            continue
        else:
            source1 = False
            source2 = False

        with GlobalVals.EKF_DATA_BUFFER_MUTEX:
            while len(GlobalVals.EKF_DATA_BUFFER) > 0:

                # get the GPS data
                objEKF = GlobalVals.EKF_DATA_BUFFER.pop(0)
                objEKF = GlobalVals.EKF_BUFFER.pop(0)

                messageStr = "{'system': " + str(objEKF.sysID) + "; 'altitude': " + str(objEKF.alt) + "; 'latitude': " + str(objEKF.lat) + "; 'longitude': " + str(objEKF.lon) + "; 'time': " + str(objEKF.epoch) + "; 'posX': " + str(objEKF.posX) + "; 'posY': " + str(objEKF.posY) +  "; 'p00': " + str(objEKF.p00) +  "; 'p01': " + str(objEKF.p01) + "; 'p10': " + str(objEKF.p10) + "; 'p11': " + str(objEKF.p11) + ";}"

                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                for i in range(GlobalVals.N_EKF_NODE_PUBLISH):
                    try:
                        Distro_Connection[i].sendall(messageStr_bytes)
                        # print("Sending RSSI:",messageStr)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error when sending to RSSI Distro_Connection[",i,"]. Now closing thread.")
                        breakThread = True
                        break
    # while True:
    #     print("Close EKF Distro to other nodes ")
    #     time.sleep(1)
    for i in range(GlobalVals.N_EKF_NODE_PUBLISH):
        Distro_Connection[i].close()
