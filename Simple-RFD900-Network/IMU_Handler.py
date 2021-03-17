import socket
import struct
import time
from threading import Thread
import threading

import GlobalVals
import CustMes 
import NetworkManager

IMU_DistroThreadLock = threading.Lock()

def extract_string_data(preString,endString,string_data):
    preIndex = string_data.find(preString)
    endIndex = string_data.find(endString, preIndex)
    return string_data[preIndex + len(preString):endIndex]

def convert_to_array(string_data):
    start_parsing = 0
    array = []
    while True:
        comma_index = string_data.find(",",start_parsing)
        if comma_index != -1:
            val = float(string_data[start_parsing:comma_index])
            array.append(val)
            start_parsing = comma_index + 1
        else:
            try:
                val = float(string_data[start_parsing:len(string_data)])
            except:
                break
            array.append(val)
            break
    return array

#=====================================================
# Thread for local IMU logger socket connection 
#=====================================================
def IMULoggerSocket():

    # set up socket
    try:
        socket_logger = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socket_logger.bind((GlobalVals.HOST, GlobalVals.IMU_LOGGER_SOCKET))
        socket_logger.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the IMU logger socket. This thread will now stop.")
        with GlobalVals.BREAK_IMU_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_IMU_LOGGER_THREAD = True
        return 

    # intialize variables 
    synced = False
    syncA = False
    syncB = False
    bufferRead = 1

    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_IMU_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_IMU_LOGGER_THREAD:
                break

        # reset buffer read when not synced 
        if not synced:
            bufferRead = 1024

        # read the socket 
        try:
            data_bytes, server = socket_logger.recvfrom(bufferRead)
            print(data_bytes)
        except:
            print("Connection error.")
            break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue
        
        raw_data = data_bytes.decode('utf-8')
        # for the sync bytes (0xAA 0x55)
        if not synced:
            if raw_data.find("{")==-1 or raw_data.find("}")==-1:
                synced = False
                continue
            else:
                synced = True
                extract_string_data("{","}",raw_data)
        # once it is scyned read the rest of the data 
        if synced:
            # store converted values 
            Epoch = float(extract_string_data("EPOCH:",";",raw_data)) 
            Acceleration_i = convert_to_array(extract_string_data("ACCELERATION:",";",raw_data))[0]
            Acceleration_j = convert_to_array(extract_string_data("ACCELERATION:",";",raw_data))[1]   
            Acceleration_k = convert_to_array(extract_string_data("ACCELERATION:",";",raw_data))[2]
            MagneticVector_i = convert_to_array(extract_string_data("MAGNETIC_VECTOR:",";",raw_data))[0]
            MagneticVector_j = convert_to_array(extract_string_data("MAGNETIC_VECTOR:",";",raw_data))[1]
            MagneticVector_k = convert_to_array(extract_string_data("MAGNETIC_VECTOR:",";",raw_data))[2]
            RawQT_w = convert_to_array(extract_string_data("RAW_QT:",";",raw_data))[0]
            RawQT_i = convert_to_array(extract_string_data("RAW_QT:",";",raw_data))[1]
            RawQT_j = convert_to_array(extract_string_data("RAW_QT:",";",raw_data))[2]
            RawQT_k = convert_to_array(extract_string_data("RAW_QT:",";",raw_data))[3]
            Euler321_psi = convert_to_array(extract_string_data("EULER_321:","}",raw_data))[0] 
            Euler321_theta = convert_to_array(extract_string_data("EULER_321:","}",raw_data))[1] 
            Euler321_phi = convert_to_array(extract_string_data("EULER_321:","}",raw_data))[2] 
            Gyroscope_i = convert_to_array(extract_string_data("GYRO:",";",raw_data))[0]
            Gyroscope_j = convert_to_array(extract_string_data("GYRO:",";",raw_data))[1]
            Gyroscope_k = convert_to_array(extract_string_data("GYRO:",";",raw_data))[2]
            print(Euler321_phi)
            print("============================")

            # Debug message 
            #print(str(GPSTime) + "," + str(Longitude) + "," + str(Latitude) + "," + str(Altitude) + "\n")  

            # use GPS message payload to store value 
            IMUData = CustMes.MESSAGE_GPS()
            IMUData.EpochTuple = Epoch
            IMUData.Acceleration_i = Acceleration_i
            IMUData.Acceleration_j = Acceleration_j
            IMUData.Acceleration_k = Acceleration_k
            IMUData.MagneticVector_i = MagneticVector_i
            IMUData.MagneticVector_j = MagneticVector_j
            IMUData.MagneticVector_k = MagneticVector_k
            IMUData.RawQT_w = RawQT_w
            IMUData.RawQT_i = RawQT_i
            IMUData.RawQT_j = RawQT_j
            IMUData.RawQT_k = RawQT_k
            IMUData.Euler321_psi = Euler321_psi
            IMUData.Euler321_theta = Euler321_theta
            IMUData.Euler321_phi = Euler321_phi
            IMUData.Gyroscope_i = Gyroscope_i
            IMUData.Gyroscope_j = Gyroscope_j
            IMUData.Gyroscope_k = Gyroscope_k
            IMUData.SystemID = GlobalVals.SYSTEM_ID

            # add data to the gps buffer 
            with GlobalVals.IMU_DATA_BUFFER_MUTEX:
                GlobalVals.IMU_DATA_BUFFER.append(IMUData)
            
            # set the flag for the data 
            with GlobalVals.RECIEVED_IMU_LOCAL_DATA_MUTEX:
                GlobalVals.RECIEVED_IMU_LOCAL_DATA = True

            # send GPS data to other balloons 
            IMUPacket = CustMes.MESSAGE_FRAME()
            IMUPacket.SystemID = GlobalVals.SYSTEM_ID
            IMUPacket.MessageID = 4
            IMUPacket.TargetID = 0
            IMUPacket.Payload = IMUData.data_to_bytes()
            NetworkManager.sendPacket(IMUPacket)

            # reset 
            synced = False

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(0.01)  

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
                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                try:
                    Thread(target=Threaded_Client, args=(Distro_Connection,messageStr_bytes))
                    # Distro_Connection.sendall(messageStr_bytes)
                except Exception as e:
                    print("Exception: " + str(e.__class__))
                    print("Error in the logger socket. Now closing thread.")
                    breakThread = True
                    break
                
    Distro_Connection.close()