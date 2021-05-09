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
        print("TEMPERATURE SOCKET")
        # if flag is set break the thread 
        with GlobalVals.BREAK_TEMP_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_TEMP_LOGGER_THREAD:
                break

        # read the socket 
        try:
            data_bytes = socket_logger.recv(bufferRead)
        except:
            print("Connection error.")
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
            print(TempData)
            print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 


#=====================================================
# Thread for distributing IMU info to other scripts 
#=====================================================
def Threaded_Client(connection,msg):
    while True:
        with IMU_DistroThreadLock:
            connection.sendall(msg)
    connection.close()

def IMUDistributor():

    # start socket 
    Distro_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Distro_Socket.bind((GlobalVals.HOST, GlobalVals.IMU_DISTRO_SOCKET))
    Distro_Socket.settimeout(GlobalVals.IMU_LOGGER_SOCKET_TIMEOUT)

    # Wait for connection on the distro socket 
    try:
        Distro_Socket.listen(1) 
        Distro_Connection, addr = Distro_Socket.accept()  
        Distro_Connection.settimeout(GlobalVals.IMU_LOGGER_SOCKET_TIMEOUT) 
        print("Logger Connected to ", addr)                                            
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.BREAK_IMU_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_IMU_DISTRO_THREAD = True
        return
    
    source1 = False
    source2 = False
    breakThread = False

    while True:

        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.BREAK_IMU_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_IMU_DISTRO_THREAD:
                break

        # check if local GPS data has been recived 
        with GlobalVals.RECIEVED_IMU_LOCAL_DATA_MUTEX:
            if GlobalVals.RECIEVED_IMU_LOCAL_DATA:
                source1 = True
                GlobalVals.RECIEVED_IMU_LOCAL_DATA = False
        
        # check if GPS data from the radio has been recieved 
        with GlobalVals.RECIEVED_IMU_RADIO_DATA_MUTEX:
            if GlobalVals.RECIEVED_IMU_RADIO_DATA:
                source2 = True
                GlobalVals.RECIEVED_IMU_RADIO_DATA = False
        
        # if no data has been recieved sleep and loop
        if not source1 and not source2:
            time.sleep(0.1)
            continue
        else:
            source1 = False
            source2 = False

        with GlobalVals.IMU_DATA_BUFFER_MUTEX:
            while len(GlobalVals.IMU_DATA_BUFFER) > 0:

                # get the GPS data
                IMUData = GlobalVals.IMU_DATA_BUFFER.pop(0)
                EpochTuple = IMUData.EpochTuple
                Acceleration_i = IMUData.Acceleration_i
                Acceleration_j = IMUData.Acceleration_j
                Acceleration_k = IMUData.Acceleration_k
                MagneticVector_i = IMUData.MagneticVector_i
                MagneticVector_j = IMUData.MagneticVector_j 
                MagneticVector_k = IMUData.MagneticVector_k 
                RawQT_w = IMUData.RawQT_w
                RawQT_i = IMUData.RawQT_i
                RawQT_j = IMUData.RawQT_j
                RawQT_k = IMUData.RawQT_k
                Euler321_psi = IMUData.Euler321_psi
                Euler321_theta = IMUData.Euler321_theta
                Euler321_phi = IMUData.Euler321_phi 
                Gyroscope_i = IMUData.Gyroscope_i
                Gyroscope_j = IMUData.Gyroscope_j
                Gyroscope_k = IMUData.Gyroscope_k

                SystemID = IMUData.SystemID

                # create message string 
                messageStr = "{'system': " + str(SystemID) + "; 'time': " + str(EpochTuple) + \
                    "; 'acceleration': " + str(Acceleration_i) + "," + str(Acceleration_j) + "," + str(Acceleration_k) +\
                    "; 'magneticVector': " + str(MagneticVector_i) + "," + str(MagneticVector_j) + "," + str(MagneticVector_k) +\
                    "; 'rawQT': " + str(RawQT_w) + "," + str(RawQT_i) + "," + str(RawQT_j) + "," + str(RawQT_k) +\
                    "; 'euler321': " + str(Euler321_psi) + "," + str(Euler321_theta) + "," + str(Euler321_phi) +\
                    "; 'gyroscope': " + str(Gyroscope_i) + "," + str(Gyroscope_j) + "," + str(Gyroscope_k) +\
                    "}"
                # print(messageStr)
                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                try:
                    # Thread(target=Threaded_Client, args=(Distro_Connection,messageStr_bytes))
                    Distro_Connection.sendall(messageStr_bytes)
                except Exception as e:
                    print("Exception: " + str(e.__class__))
                    print("Error in the logger socket. Now closing thread.")
                    breakThread = True
                    break
                
    Distro_Connection.close()