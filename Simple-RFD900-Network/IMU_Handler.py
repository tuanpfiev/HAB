import socket
import struct
import time
from threading import Thread
import threading

import GlobalVals
import CustMes 
import NetworkManager

IMU_DistroThreadLock = threading.Lock()

#=====================================================
# Thread for local IMU logger socket connection 
#=====================================================
def IMULoggerSocket():

    # set up socket
    try:
        socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        socket_logger.connect((GlobalVals.HOST, GlobalVals.IMU_LOGGER_SOCKET))
        socket_logger.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the logger socket. This thread will now stop.")
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
            bufferRead = 1

        # read the socket 
        try:
            data_bytes = socket_logger.recv(bufferRead)
        except:
            print("Connection error.")
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
                bufferRead = 136
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
        if synced and bufferRead == 136:

            # convert payload values back to double
            EpochTuple = struct.unpack('!d',data_bytes[0:8])
            Acceleration_iTuple = struct.unpack('!d',data_bytes[8:16])
            Acceleration_jTuple = struct.unpack('!d',data_bytes[16:24])   
            Acceleration_kTuple = struct.unpack('!d',data_bytes[24:32])
            MagneticVector_iTuple = struct.unpack('!d',data_bytes[32:40])
            MagneticVector_jTuple = struct.unpack('!d',data_bytes[40:48])
            MagneticVector_kTuple = struct.unpack('!d',data_bytes[48:56])
            RawQT_iTuple = struct.unpack('!d',data_bytes[56:64])
            RawQT_jTuple = struct.unpack('!d',data_bytes[64:72])
            RawQT_kTuple = struct.unpack('!d',data_bytes[72:80])
            RawQT_wTuple = struct.unpack('!d',data_bytes[80:88])
            Euler321_phiTuple = struct.unpack('!d',data_bytes[88:96])
            Euler321_thetaTuple = struct.unpack('!d',data_bytes[96:104])
            Euler321_psiTuple = struct.unpack('!d',data_bytes[104:112])
            Gyroscope_iTuple = struct.unpack('!d',data_bytes[112:120])
            Gyroscope_jTuple = struct.unpack('!d',data_bytes[120:128])
            Gyroscope_kTuple = struct.unpack('!d',data_bytes[128:136])

            # store converted values 
            Epoch = EpochTuple[0] 
            Acceleration_i = Acceleration_iTuple[0] 
            Acceleration_j = Acceleration_jTuple[0]    
            Acceleration_k = Acceleration_kTuple[0] 
            MagneticVector_i = MagneticVector_iTuple[0] 
            MagneticVector_j = MagneticVector_jTuple[0] 
            MagneticVector_k = MagneticVector_kTuple[0] 
            RawQT_w = RawQT_wTuple[0]
            RawQT_i = RawQT_iTuple[0]
            RawQT_j = RawQT_jTuple[0] 
            RawQT_k = RawQT_kTuple[0] 
            Euler321_psi = Euler321_psiTuple[0] 
            Euler321_theta = Euler321_thetaTuple[0] 
            Euler321_phi = Euler321_phiTuple[0] 
            Gyroscope_i = Gyroscope_iTuple[0]
            Gyroscope_j = Gyroscope_jTuple[0]
            Gyroscope_k = Gyroscope_kTuple[0]

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