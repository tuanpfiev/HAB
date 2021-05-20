import socket
import struct
import time
from threading import Thread
import threading
from _thread import *
import GlobalVals
import CustMes 
import NetworkManager

import sys, os

sys.path.insert(1,'../utils')
from utils import get_port
from common import *
from common_class import *
#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
def GPSLoggerSocket():

    # set up socket
    try:
        socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        socket_logger.connect((GlobalVals.HOST, GlobalVals.GPS_LOGGER_SOCKET))
        socket_logger.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the GPSLoggerSocket logger socket. This thread will now stop.")
        print("GPS_PORT: ",GlobalVals.GPS_LOGGER_SOCKET)
        print("GPS_TIMEOUT: ",GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT )
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_LOGGER_THREAD = True
        return 

    # intialize variables 
    synced = False
    syncA = False
    syncB = False
    bufferRead = 1
    breakMainThread = False
    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_LOGGER_THREAD:
                break

        # reset buffer read when not synced 
        if not synced:
            bufferRead = 1

        # read the socket 
        while True:
            try:
                data_bytes = socket_logger.recv(bufferRead)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("GPSLoggerSocket receive timed out. Retrying ...")
                    time.sleep(0.1)
                    continue
                else:
                    print("GPSLoggerSocket: Receive Connection error.")
                    breakMainThread = True
                    break
        
        if breakMainThread:
            break
            
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue
        
        # for the sync bytes (0xAA 0x55)
        if not synced:
            if data_bytes[0] == 0xAA and not syncA:
                syncA = True
                bufferRead = 1
                continue
            elif data_bytes[0] == 0x55 and syncA:
                syncB = True
                bufferRead = 32
            else:
                syncA = False
                syncB = False
                bufferRead = 1
                continue 
            
            # if both bytes have been found in order then the socket buffer is synced 
            if syncA and syncB:
                synced = True
                syncA = False
                syncB = False
                continue 
        
        # once it is scyned read the rest of the data 
        if synced and bufferRead == 32:

            # convert payload values back to double
            LongitudeTuple = struct.unpack('!d',data_bytes[0:8])
            LatitudeTuple = struct.unpack('!d',data_bytes[8:16])
            AltitudeTuple = struct.unpack('!d',data_bytes[16:24])
            GPSTimeTuple = struct.unpack('!d',data_bytes[24:32])

            # store converted values 
            Longitude = LongitudeTuple[0]
            Latitude = LatitudeTuple[0]
            Altitude = AltitudeTuple[0]
            GPSTime = GPSTimeTuple[0]

            # Debug message 
            #print(str(GPSTime) + "," + str(Longitude) + "," + str(Latitude) + "," + str(Altitude) + "\n")  

            # use GPS message payload to store value 
            GPSData = CustMes.MESSAGE_GPS()
            GPSData.Longitude = Longitude
            GPSData.Latitude = Latitude
            GPSData.Altitude = Altitude
            GPSData.GPSTime = GPSTime
            GPSData.SystemID = GlobalVals.SYSTEM_ID

            # add data to the gps buffer 
            with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                GlobalVals.GPS_DATA_BUFFER.append(GPSData)
            
            # set the flag for the data 
            with GlobalVals.RECIEVED_GPS_LOCAL_DATA_MUTEX:
                GlobalVals.RECIEVED_GPS_LOCAL_DATA = True

            # send GPS data to other balloons 
            GPSPacket = CustMes.MESSAGE_FRAME()
            GPSPacket.SystemID = GlobalVals.SYSTEM_ID
            GPSPacket.MessageID = 2
            GPSPacket.TargetID = 0
            GPSPacket.Payload = GPSData.data_to_bytes()
            NetworkManager.sendPacket(GPSPacket)
            # print("SEND GPS TO RFD900!!!!!!")
            # reset 
            synced = False

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(0.01)  

    socket_logger.close()
    return 

#=====================================================
# Thread for distributing GPS info to other scripts 
#=====================================================


def GPSDistributor():

    Distro_Socket = [None]*GlobalVals.N_NODE_PUBLISH
    Distro_Connection = [None]*GlobalVals.N_NODE_PUBLISH
    for i in range(GlobalVals.N_NODE_PUBLISH):
        # start socket 
        Distro_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Distro_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
        Distro_Socket[i].bind((GlobalVals.HOST, GlobalVals.GPS_DISTRO_SOCKET[i]))
        Distro_Socket[i].settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
        

        # Wait for connection on the distro socket 
        try:
            Distro_Socket[i].listen(1) 
            Distro_Connection[i], addr = Distro_Socket[i].accept()  
            Distro_Connection[i].settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT) 
            print("Logger[",i,"] Connected to ", addr)                                            
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error in the GPSDistributor[",i,"] logger socket. Now closing thread.")
            with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_GPS_DISTRO_THREAD = True
            return 0
  
    
    source1 = False
    source2 = False
    breakThread = False

    while True:

        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_DISTRO_THREAD:
                break

        # check if local GPS data has been recived 
        with GlobalVals.RECIEVED_GPS_LOCAL_DATA_MUTEX:
            if GlobalVals.RECIEVED_GPS_LOCAL_DATA:
                source1 = True
                GlobalVals.RECIEVED_GPS_LOCAL_DATA = False
        
        # check if GPS data from the radio has been recieved 
        with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
            if GlobalVals.RECIEVED_GPS_RADIO_DATA:
                source2 = True
                GlobalVals.RECIEVED_GPS_RADIO_DATA = False
        
        # if no data has been recieved sleep and loop
        if not source1 and not source2:
            time.sleep(0.1)
            continue
        else:
            source1 = False
            source2 = False

        with GlobalVals.GPS_DATA_BUFFER_MUTEX:
            while len(GlobalVals.GPS_DATA_BUFFER) > 0:

                # get the GPS data
                GPSData = GlobalVals.GPS_DATA_BUFFER.pop(0)
                Longitude = GPSData.Longitude
                Latitude = GPSData.Latitude
                Altitude = GPSData.Altitude
                GPSTime = int(GPSData.GPSTime)
                SystemID = GPSData.SystemID

                # create message string 
                messageStr = "{'system': " + str(SystemID) + "; 'altitude': " + str(Altitude) + "; 'latitude': " + str(Latitude) + "; 'longitude': " + str(Longitude) + "; 'time': " + str(GPSTime) + "}"
                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                for i in range(GlobalVals.N_NODE_PUBLISH):
                    try:
                        Distro_Connection[i].sendall(messageStr_bytes)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error when sending to Distro_Connection[",i,"]. Now closing thread.")
                        breakThread = True
                        break
                
    for i in range(GlobalVals.N_NODE_PUBLISH):
        Distro_Connection[i].close()


def GPS_FormatCheck(GPSdata):
    errString = []
    err = False
    if not GPSdata.SystemID in GlobalVals.REAL_BALLOON:
        errString.append("GPSdata.SystemID: " + str(GPSdata.SystemID))
        err = True
    
    if GPSdata.SystemID == GlobalVals.SYSTEM_ID:
        errString.append("GPSdata.SystemID must be different")
        err = True

    if not valueInRange(GPSdata.Longitude,[-180,180]):
        errString.append("Longitude: " + str(GPSdata.Longitude))
        err = True

    if not valueInRange(GPSdata.Latitude,[-90,90]):
        errString.append("Latitude: " + str(GPSdata.Latitude))
        err = True
    
    if not valueInRange(GPSdata.Altitude,[-100,50000]):
        errString.append("Altitude: ",GPSdata.Altidue)
        err = True

    if not valueInRange(GPSdata.GPSTime,[GlobalVals.EXPERIMENT_TIME,None]):
        errString.append("Epoch: " + str(GPSdata.GPSTime))
        err = True
    if err:
        print(errString)
        return False
    else:
        return True

