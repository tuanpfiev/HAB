# import libraries 
import time
from threading import Thread

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter
import GPSHandler
import PingLogger
import ImaginaryBalloons
import IMU_Handler
import EKFHandler
import RSSI_Handler
import TemperatureHandler
import sys, os

sys.path.insert(1,'../utils')
from utils import get_port
from common import *
from common_class import *

#=====================================================
# Main function  
#=====================================================
def main():
    
    recievedPackets = False
    # sendTime = int(time.time() + 1)

    # this loop will wait for packets and then process them 
    while True:
        
        # curTime = int(time.time())
        
        # # send dummy GPS data once a second  
        # if curTime >= sendTime:
            
        #     sendTime = curTime + 1
        #     GPSpacket = CustMes.MESSAGE_FRAME()
        #     GPSdata = CustMes.MESSAGE_GPS()

        #     # set up GPS packet 
        #     GPSpacket.MessageID = 0x02
        #     GPSpacket.TargetID = 0
        #     GPSpacket.SystemID = GlobalVals.SYSTEM_ID

        #     # load dummy GPS data
        #     GPSdata.Longitude = 7.7
        #     GPSdata.Latitude = 8.8
        #     GPSdata.Altitude = 9.9
        #     GPSdata.GPSTime = 10.1

        #     # send dummy GPS packet
        #     GPSpacket.Payload = GPSdata.data_to_bytes()
        #     NetworkManager.sendPacket(GPSpacket)


        # check if packets have been recived 
        with GlobalVals.RECIEVED_PACKETS_MUTEX:
            if GlobalVals.RECIEVED_PACKETS:
                recievedPackets = True
        
        # if no packets have been recived then sleep and loop
        if not recievedPackets:
            time.sleep(0.1)
            continue
        else:
            recievedPackets = False
        
        # go through all the packets in the buffer 
        recievedPacket = CustMes.MESSAGE_FRAME()
        with GlobalVals.PACKET_BUFFER_IN_MUTEX:
            while len(GlobalVals.PACKET_BUFFER_IN) > 0:

                recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(0)

                # if the packet is a ping 
                if recievedPacket.MessageID == 1:
                    NetworkManager.PingRespond(recievedPacket.SystemID,recievedPacket.Timestamp)

                # if the packet is a GPS data packet 
                if recievedPacket.MessageID == 2:
                    
                    # get the GPS data
                    GPSdata = CustMes.MESSAGE_GPS()                    
                    error = GPSdata.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: GPS data error " + str(error) + ".\n")
                        continue
                    
                    

                    # print("GPS Data from " + str(recievedPacket.SystemID) + ":" +"Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,1)) + ", Time:" + str(round(GPSdata.GPSTime,1)))
                    # print("Lon:" + str(GPSdata.Longitude) + ", Lat:" + str(GPSdata.Latitude) + ", Alt:" + str(GPSdata.Altitude) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # set the system id for the GPS data
                    GPSdata.SystemID = recievedPacket.SystemID

                    if not GPSHandler.GPS_FormatCheck(GPSdata):
                        print("GPS message via RFD900 was broken. Discard it...") 
                        continue

                    # put data into the buffer
                    with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                        GlobalVals.GPS_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_GPS_RADIO_DATA = True
                    
                    continue

                    # get the GPS data  
                    # recData = CustMes.MESSAGE_GPS()                 
                    # error = recData.bytes_to_data(recievedPacket.Payload)
                    # if error != 0:
                    #     print ("Radio Network Main: GPS data error " + str(error) + ".\n")
                    #     continue
                    
                    # print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                    # print("Lon:" + str(recData.Longitude) + ", Lat:" + str(recData.Latitude) + ", Alt:" + str(recData.Altitude) + ", Time:" + str(recData.GPSTime) + "\n") 
                    
                    # continue
                
                # if the packet is string message  
                if recievedPacket.MessageID == 3: 

                    # get the string  
                    StrData = CustMes.MESSAGE_STR()                    
                    error = StrData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Packet error, packet will be discarded.\n")
                        continue
                        
                    # print string 
                    print(StrData.MessageStr)

                    continue

                if recievedPacket.MessageID == 4:
                    
                    # get the GPS data
                    IMUdata = CustMes.MESSAGE_IMU()                    
                    error = IMUdata.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: IMU data error " + str(error) + ".\n")
                        continue
                    
                    print("IMU Data from " + str(recievedPacket.SystemID) + ":")
                    print("Euler:" + str(IMUdata.Euler321_theta) + "\n")

                    # set the system id for the GPS data
                    IMUdata.SystemID = recievedPacket.SystemID

                    # put data into the buffer
                    with GlobalVals.IMU_DATA_BUFFER_MUTEX:
                        GlobalVals.IMU_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_IMU_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_IMU_RADIO_DATA = True
                    
                    continue

                if recievedPacket.MessageID == 5:
                    
                    # get the GPS data
                    EKF_Data = CustMes.MESSAGE_EKF()                    
                    error = EKF_Data.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: EKF data error " + str(error) + ".\n")
                        continue
                    

                    # print("GPS Data from " + str(recievedPacket.SystemID) + ":" +"Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,1)) + ", Time:" + str(round(GPSdata.GPSTime,1)))
                    # print("Lon:" + str(GPSdata.Longitude) + ", Lat:" + str(GPSdata.Latitude) + ", Alt:" + str(GPSdata.Altitude) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # set the system id for the GPS data
                    EKF_Data.SystemID = recievedPacket.SystemID

                    if not EKFHandler.EKF_FormatCheck(EKF_Data):
                        print("EKF message via RFD900 was broken. Discard it...") 
                        continue

                    print("EKF Data from [",EKF_Data.SystemID,"], Lat: ", EKF_Data.Latitude, ", Lon: ", EKF_Data.Longitude, ", Alt: ", EKF_Data.Altitude)
                    # put data into the buffer
                    # with GlobalVals.EKF_DATA_BUFFER_MUTEX:
                    #     GlobalVals.EKF_DATA_BUFFER.append(EKF_Data)

                    # set the flags for the buffer 
                    # with GlobalVals.RECIEVED_EKF_RADIO_DATA_MUTEX:
                    #     GlobalVals.RECIEVED_EKF_RADIO_DATA = True
                    
                    continue
                # Temperature
                if recievedPacket.MessageID == 6:

                    # get the RSSI data
                    temperatureData = CustMes.MESSAGE_TEMP()                    
                    error = temperatureData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: temperature data error " + str(error) + ".\n")
                        continue
                    
                    # set the system id for the GPS data
                    temperatureData.SystemID = recievedPacket.SystemID
                    
                    if not temperatureData.SystemID in GlobalVals.REAL_BALLOON or not valueInRange(temperatureData.Temperature,[-100,150]) or not valueInRange(temperatureData.Epoch,[GlobalVals.EXPERIMENT_TIME,None]):
                        print("Temperature message via RFD900 was broken. Discard it...")
                        continue
                    
                    # print(" Temperature Data from " + str(recievedPacket.SystemID) + ":" + "Temperature:" + str(round(temperatureData.Temperature,1)))



                # RSSI
                if recievedPacket.MessageID == 7:

                    # get the RSSI data
                    RSSI_Data = CustMes.MESSAGE_RSSI()                    
                    error = RSSI_Data.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: RSSI data error " + str(error) + ".\n")
                        continue
                    
                    # set the system id for the GPS data
                    RSSI_Data.SystemID = recievedPacket.SystemID
                    # print(RSSI_Data.SystemID)
                    # print(RSSI_Data.TargetPayloadID)
                    # print(GlobalVals.RSSI_ALLOCATION)

                    # Check if the message was sent correctly via the RFD900
                    if not RSSI_Handler.RSSI_FormatCheck(RSSI_Data):
                        print("RSSI message via RFD900 was broken. Discard it...")
                        continue
                    
                    # print("RSSI Data from " + str(recievedPacket.SystemID) + ":" + "RSSI Distance:" + str(RSSI_Data.Distance) + "Filtered RSSI: " + str(RSSI_Data.FilteredRSSI) + "TargetPayloadID: " + str(RSSI_Data.TargetPayloadID) + "Time: " + str(RSSI_Data.Epoch) + "SysID: " + str(RSSI_Data.SystemID))

                    if GlobalVals.SYSTEM_ID == 1:
                        with GlobalVals.RSSI_ALLOCATION_MUTEX:
                            # print("UPDATE RSSI ALLOCATION FROM RADIO [",RSSI_Data.SystemID,"] !!!!")
                            # print(GlobalVals.RSSI_ALLOCATION)
                            GlobalVals.RSSI_ALLOCATION[RSSI_Data.SystemID-1][int(RSSI_Data.TargetPayloadID)-1] = True
                            # print("check 32")
                            # print(GlobalVals.RSSI_ALLOCATION)
                            RSSI_Handler.getPairAllocation()

                       
                    # put data into the buffer
                    with GlobalVals.RSSI_DATA_BUFFER_MUTEX:
                        GlobalVals.RSSI_DATA_BUFFER.append(RSSI_Data)

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_RSSI_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_RSSI_RADIO_DATA = True
                
                    continue

                if recievedPacket.MessageID == 8:

                    # get the RSSI data
                    RSSI_AllocationData = CustMes.MESSAGE_RSSI_ALLOCATION()                    
                    error = RSSI_AllocationData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: RSSI Allocation data error " + str(error) + ".\n")
                        continue
                    
                    # set the system id for the GPS data
                    RSSI_AllocationData.SystemID = recievedPacket.SystemID
                    
                    if not RSSI_AllocationData.SystemID in GlobalVals.REAL_BALLOON or not RSSI_AllocationData.Pair in GlobalVals.LORA_PAIR_NUM:
                        print("RSSI Allocation message via RFD900 was broken. Discard it...")
                        continue
                    
                    print(" RSSI Allocation Data from " + str(recievedPacket.SystemID) + ":" + "Pair:" + str(int(RSSI_AllocationData.Pair)))

                    # put data into the buffer
                    with GlobalVals.RSSI_DATA_ALLOCATION_BUFFER_MUTEX:
                        if len(GlobalVals.RSSI_DATA_ALLOCATION_BUFFER)>5:
                            GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.pop(0)
                        GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.append(int(RSSI_AllocationData.Pair))

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA = True
                
                    continue

#=====================================================
# Thread starter 
#=====================================================
if __name__ == '__main__':

    numArgs = len(sys.argv)
    if numArgs == 2:
        GlobalVals.SYSTEM_ID = int(sys.argv[1])

    print('SystemID is: ', GlobalVals.SYSTEM_ID)
    # set Port
    GlobalVals.PORT=get_port('RFD900')
    print('PORT: '+ GlobalVals.PORT)

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    GlobalVals.ERROR_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ErrorLog.txt"
    GlobalVals.PING_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-PingLog.txt"
    GlobalVals.PACKET_STATS_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-PacketStats.txt"
    GlobalVals.GROUND_STATION_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-GSLog.txt"


    # Start serial thread 
    NetworkThread = Thread(target=NetworkManager.SerialManagerThread,args=())
    NetworkThread.daemon = True
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.start()

    # Start logger socket
    GPSLoggerThread = Thread(target=GPSHandler.GPSLoggerSocket, args=())
    GPSLoggerThread.start()

    # Start GPS distributor 
    GPSDistroThread = Thread(target=GPSHandler.GPSDistributor, args=())
    GPSDistroThread.start()

    # Start Ping Logger 
    PingLoggerThread = Thread(target=PingLogger.PingLoggerThread, args=())
    PingLoggerThread.start()

    # Start imaginary balloon socket
    ImaginaryBalloonsThread = Thread(target=ImaginaryBalloons.ImaginaryBalloons, args=())
    ImaginaryBalloonsThread.start()

    # # Start IMU socket
    # IMULoggerThread = Thread(target=IMU_Handler.IMULoggerSocket, args=())
    # IMULoggerThread.start()

    #  # Start IMU Distro socket
    # IMUDistroThread = Thread(target=IMU_Handler.IMUDistributor, args=())
    # IMUDistroThread.start()

    # start EKF logger
    EKF_Thread = Thread(target=EKFHandler.EKFLoggerSocket, args = ())
    EKF_Thread.start()

    # start EKF distro:
    # EKF_DistroThread = Thread(target = EKFHandler.EKF_AllDistributor, args = ())
    # EKF_DistroThread.start()

    # start RSSI logger
    RSSI_Thread = [None]*(GlobalVals.N_REAL_BALLOON-1)
    for i in range(GlobalVals.N_REAL_BALLOON-1):
        RSSI_Thread[i] = Thread(target=RSSI_Handler.RSSI_LoggerSocket, args = (GlobalVals.HOST,GlobalVals.RSSI_LOGGER_SOCKET[i],i))
        RSSI_Thread[i].start()

    RSSI_DistroThread = Thread(target=RSSI_Handler.RSSI_Distributor,args = ())
    RSSI_DistroThread.start()

    RSSI_AllocationThread = Thread(target=RSSI_Handler.RSSI_AllocationDistributor,args = ())
    RSSI_AllocationThread.start()

    tempThread = Thread(target=TemperatureHandler.TemperatureLoggerSocket, args = ())
    tempThread.start()

    try:
        main()
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")

    # Safely end the serial thread
    if NetworkThread.is_alive():
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            GlobalVals.BREAK_NETWORK_THREAD = True
        NetworkThread.join()
    
    # Safely end the error thread 
    if ErrorThread.is_alive():
        with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
            GlobalVals.BREAK_ERROR_THREAD = True
        ErrorThread.join()
    
    # Safely end the logger socket thread 
    if GPSLoggerThread.is_alive():
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_LOGGER_THREAD = True
        GPSLoggerThread.join()
    
    # Safely end the GPS distributor thread 
    if GPSDistroThread.is_alive():
        with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_DISTRO_THREAD = True
        GPSDistroThread.join()
    
    # Safely end the ping logger thread 
    if PingLoggerThread.is_alive():
        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            GlobalVals.BREAK_PING_THREAD = True
        PingLoggerThread.join()
    
    if ImaginaryBalloonsThread.is_alive():
        with GlobalVals.BREAK_IMAGINARY_BALLOONS_MUTEX:
            GlobalVals.BREAK_IMAGINARY_BALLOONS_THREAD = True
        ImaginaryBalloonsThread.join()

    # if IMULoggerThread.is_alive():
    #     with GlobalVals.BREAK_IMU_LOGGER_THREAD_MUTEX:
    #         GlobalVals.BREAK_IMU_LOGGER_THREAD = True
    #     IMULoggerThread.join()

    # if IMUDistroThread.is_alive():
    #     with GlobalVals.BREAK_IMU_DISTRO_THREAD_MUTEX:
    #         GlobalVals.BREAK_IMU_DISTRO_THREAD = True
    #     IMUDistroThread.join()

    if EKF_Thread.is_alive():
        with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_EKF_LOGGER_THREAD = True
        EKF_Thread.join()

    # if EKF_DistroThread.is_alive():
    #     with GlobalVals.BREAK_EKF_DISTRO_THREAD_MUTEX:
    #         GlobalVals.BREAK_EKF_DISTRO_THREAD = True
    #     EKF_Thread.join()

    for i in range(GlobalVals.N_REAL_BALLOON-1):
        if RSSI_Thread[i].is_alive():
            with GlobalVals.BREAK_RSSI_LOGGER_THREAD_MUTEX[i]:
                GlobalVals.BREAK_RSSI_LOGGER_THREAD[i] = True
            RSSI_Thread[i].join()

    if RSSI_DistroThread.is_alive():
        with GlobalVals.BREAK_RSSI_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_DISTRO_THREAD = True

    if RSSI_AllocationThread.is_alive():
        with GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD = True
    if tempThread.is_alive():
        with GlobalVals.BREAK_TEMP_LOGGER_THREAD_MUTEX:
            GlobalVals.BREAK_TEMP_LOGGER_THREAD = True
        tempThread.join()

    
    
