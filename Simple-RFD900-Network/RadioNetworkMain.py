# import libraries 
import time
from threading import Thread
from copy import deepcopy
import sys, os
import gc

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

sys.path.insert(1,'../utils')
from utils import get_port
from common import *
from common_class import *

#=====================================================
# Main function  
#=====================================================
def main():

    recievedPackets = False

    # this loop will wait for packets and then process them 
    while True:

        # check if packets have been recived 
        with GlobalVals.RECIEVED_PACKETS_MUTEX:
            if GlobalVals.RECIEVED_PACKETS:
                GlobalVals.RECIEVED_PACKETS = False
                recievedPackets = True
        
        # if packets have been recieved 
        if recievedPackets:
            recievedPackets = False
            

            # go through all the packets in the buffer 
            while True:
                recievedPacket = [[] for _ in range(15)]
                with GlobalVals.PACKET_BUFFER_IN_MUTEX:
                    for i in range(len(GlobalVals.PACKET_BUFFER_IN)):
                        if len(GlobalVals.PACKET_BUFFER_IN[i]) > 0:
                            recievedPacket[i] = GlobalVals.PACKET_BUFFER_IN[i].pop(0)
                        # else:
                        #     break

                
                # if the packet is a ping 
                # if recievedPacket.MessageID == 1:
                if recievedPacket[1]:
                    
                    # get the payload of the ping
                    pingPayload = CustMes.MESSAGE_PING()
                    error = pingPayload.bytes_to_data(recievedPacket[1].Payload)

                    # if there was an error with the payload report it 
                    if error != 0:
                        print("Ping Data Error: RadioNetworkMain:" + str(error))
                        print(recievedPacket[1].Payload.hex())
                        print(recievedPacket[1].SystemID)
                        print('\n')
                    
                    else:
                    
                        # check if the ping is an intiator 
                        if pingPayload.Intiator:
                            
                            # respond to the ping if it is  
                            NetworkManager.PingRespond(recievedPacket[1].SystemID,recievedPacket[1].Timestamp)
                        
                        else:

                            # send ping to ping thread if it isn't 
                            with GlobalVals.PACKET_PING_BUFFER_MUTEX:
                                GlobalVals.PACKET_PING_BUFFER.append(recievedPacket[1])

                            # set the recieved flag
                            with GlobalVals.RECIEVED_PING_MUTEX:
                                GlobalVals.RECIEVED_PING = True
                    

                # if the packet is a GPS data packet 
                # if recievedPacket.MessageID == 2:
                if recievedPacket[2]:                 
                    # get the GPS data
                    GPSdata = CustMes.MESSAGE_GPS()                    
                    error = GPSdata.bytes_to_data(recievedPacket[2].Payload)

                    # report any errors
                    if error != 0:
                        print("GPS Data Error: RadioNetworkMain:" + str(error))
                        # print(recievedPacket[2].Payload.hex())
                        # print(recievedPacket[2].SystemID)
                        # print('\n')
                    else:
                        # print("gps packet ok: ",recievedPacket[2].Payload.hex())

                        # print GPS data to screen 
                        print("GPS Data from " + str(recievedPacket[2].SystemID) + ":")
                        print("Lon:" + str(GPSdata.Longitude) + ", Lat:" + str(GPSdata.Latitude) + ", Alt:" + str(GPSdata.Altitude) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                        # set the system id for the GPS data
                        GPSdata.SystemID = recievedPacket[2].SystemID
        
                        # put data into the buffer
                        with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                            GlobalVals.GPS_DATA_BUFFER.append(GPSdata)

                        # set the flags for the buffer 
                        with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
                            GlobalVals.RECIEVED_GPS_RADIO_DATA = True
                    
                
                # if the packet is string message  
                # if recievedPacket.MessageID == 3: 
                if recievedPacket[3]:
                    # get the string  
                    StrData = CustMes.MESSAGE_STR()                    
                    error = StrData.bytes_to_data(recievedPacket[3].Payload)
                    if error != 0:
                        print ("Packet error, packet will be discarded.\n")
                    else:
                        # print string 
                        print(StrData.MessageStr)
                

                # IMU msg:
                # IMU_MsgID = 4
                # if recievedPacket[IMU_MsgID]:
                #     IMU_Data = CustMes.MESSAGE_IMU()
                #     error = IMU_Data.bytes_to_data(recievedPacket[IMU_MsgID].Payload)
                #     if error != 0:
                #         print("IMU Data Error: RadioNetworkMain:" + str(error))
                #     else:
                #         print("IMU Data from " + str(recievedPacket.SystemID) + ":")
                #         print("Euler:" + str(IMU_Data.Euler321_theta) + "\n")

                #         # set the system id for the GPS data
                #         IMU_Data.SystemID = recievedPacket.SystemID

                #         # put data into the buffer
                #         with GlobalVals.IMU_DATA_BUFFER_MUTEX:
                #             GlobalVals.IMU_DATA_BUFFER.append(GPSdata)

                #         # set the flags for the buffer 
                #         with GlobalVals.RECIEVED_IMU_RADIO_DATA_MUTEX:
                #             GlobalVals.RECIEVED_IMU_RADIO_DATA = True
                
                # EKF msg
                EKF_MsgID = 5
                if recievedPacket[EKF_MsgID]:
                    EKF_Data = CustMes.MESSAGE_EKF()
                    error = EKF_Data.bytes_to_data(recievedPacket[EKF_MsgID].Payload)
                    if error != 0:
                        print("EKF_Data Error: RadioNetworkMain:" + str(error))
                    else:
                        EKF_Data.SystemID = recievedPacket.SystemID

                        if not EKFHandler.EKF_FormatCheck(EKF_Data):
                            print("EKF message via RFD900 was broken. Discard it...") 
                            continue

                    print("EKF EKF EKF Data from [",EKF_Data.SystemID,"], Lat: ", EKF_Data.Latitude, ", Lon: ", EKF_Data.Longitude, ", Alt: ", EKF_Data.Altitude)
                    # put data into the buffer

                # Temperature msg
                temperatureMsgID = 6
                if recievedPacket[temperatureMsgID]:
                    temperatureData = CustMes.MESSAGE_TEMP()
                    error = temperatureData.bytes_to_data(recievedPacket[temperatureMsgID].Payload)
                    if error != 0:
                        print("Temperature_Data Error: RadioNetworkMain:" + str(error))
                    else:
                        temperatureData.SystemID = recievedPacket.SystemID
                    
                    if not TemperatureHandler.temperatureFormatCheck(temperatureData):
                        print("Temperature message via RFD900 was broken. Discard it...")
                        continue
                
                RSSI_MsgID = 7
                if recievedPacket[RSSI_MsgID]:
                    RSSI_Data = CustMes.MESSAGE_RSSI()
                    error = RSSI_Data.bytes_to_data(recievedPacket[RSSI_MsgID].Payload)
                    if error != 0:
                        print("RSSI_Data Error: RadioNetworkMain:" + str(error))
                    else:
                        RSSI_Data.SystemID = recievedPacket.SystemID
                        # print(RSSI_Data.SystemID)
                        # print(RSSI_Data.TargetPayloadID)
                        # print(GlobalVals.RSSI_ALLOCATION)

                        # Check if the message was sent correctly via the RFD900
                        if not RSSI_Handler.RSSI_FormatCheck(RSSI_Data):
                            print("RSSI message via RFD900 was broken. Discard it...")
                            continue
                        
                        print("RSSI Data from " + str(recievedPacket.SystemID) + ":" + "RSSI Distance:" + str(RSSI_Data.Distance) + "Filtered RSSI: " + str(RSSI_Data.FilteredRSSI) + "TargetPayloadID: " + str(RSSI_Data.TargetPayloadID) + "Time: " + str(RSSI_Data.Epoch) + "SysID: " + str(RSSI_Data.SystemID))

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

                RSSI_AllocationID = 8
                if recievedPacket[RSSI_AllocationID]:
                    RSSI_AllocationData = CustMes.MESSAGE_RSSI_ALLOCATION()
                    error = RSSI_AllocationData.bytes_to_data(recievedPacket[RSSI_AllocationID].Payload)
                    if error != 0:
                        print("RSSI_AllocationData Error: RadioNetworkMain:" + str(error))
                    else:
                        RSSI_AllocationData.SystemID = recievedPacket.SystemID
                    
                        if not RSSI_Handler.RSSI_AllocationFormatCheck(RSSI_AllocationData):
                            print("RSSI Allocation message via RFD900 was broken. Discard it...")
                            continue
                        
                        print(" RSSI Allocation Data from " + str(recievedPacket.SystemID) + ":" + "Pair:" + str(int(RSSI_AllocationData.Pair)))

                        # put data into the buffer
                        with GlobalVals.RSSI_DATA_ALLOCATION_BUFFER_MUTEX:
                            # if len(GlobalVals.RSSI_DATA_ALLOCATION_BUFFER)>5:
                            #     GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.pop(0)
                            GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.append(int(RSSI_AllocationData.Pair))

                        # set the flags for the buffer 
                        with GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA_MUTEX:
                            GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA = True


                del recievedPacket
                gc.collect()

                


            # if this is the ground station do the following
            if GlobalVals.IS_GROUND_STATION:

                # if radio GPS data has been recived record it 
                if GlobalVals.RECIEVED_GPS_RADIO_DATA:

                    logString = ""
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
                            logString = logString + str(GPSTime) + "," + str(SystemID) + "," + str(Longitude) + "," + str(Latitude) + "," + str(Altitude) + "\n"

                    # write the log string to file  
                    try:
                        fileObj = open(GlobalVals.GROUND_STATION_LOG_FILE, "a")
                        fileObj.write(logString)
                        fileObj.close()
                    except Exception as e:
                        print("Exception: RadioNetworkMain: Log File Write: " + str(e.__class__))
                        print("RadioNetworkMain thread will continue.")



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
    NetworkThread = Thread(target=NetworkManager.RFD900_ManagerThread,args=())
    NetworkThread.setDaemon(True)
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.setDaemon(True)
    ErrorThread.start()

    # Start Ping Logger 
    PingLoggerThread = Thread(target=PingLogger.PingLoggerThread, args=())
    PingLoggerThread.setDaemon(True)
    PingLoggerThread.start()

    # if this isn't the ground station 
    if not GlobalVals.IS_GROUND_STATION:
    
        # Start logger socket
        GPSLoggerThread = Thread(target=GPSHandler.GPSLoggerSocket, args=())
        GPSLoggerThread.setDaemon(True)
        GPSLoggerThread.start()

        # Start GPS distributor 
        GPSDistroThread = Thread(target=GPSHandler.GPSDistributor, args=())
        GPSDistroThread.setDaemon(True)
        GPSDistroThread.start()

        # Start imaginary balloon socket
        ImaginaryBalloonsThread = Thread(target=ImaginaryBalloons.ImaginaryBalloons, args=())
        ImaginaryBalloonsThread.setDaemon(True)
        ImaginaryBalloonsThread.start()

        # start EKF logger
        EKF_Thread = Thread(target=EKFHandler.EKFLoggerSocket, args = ())
        EKF_Thread.setDaemon(True)
        EKF_Thread.start()

        # start EKF distro:
        EKF_DistroThread = Thread(target = EKFHandler.EKF_AllDistributor, args = ())
        EKF_DistroThread.setDaemon(True)
        EKF_DistroThread.start()

        # start RSSI logger
        RSSI_Thread = [None]*(GlobalVals.N_REAL_BALLOON-1)
        for i in range(GlobalVals.N_REAL_BALLOON-1):
            RSSI_Thread[i] = Thread(target=RSSI_Handler.RSSI_LoggerSocket, args = (GlobalVals.HOST,GlobalVals.RSSI_LOGGER_SOCKET[i],i))
            RSSI_Thread[i].setDaemon(True)
            RSSI_Thread[i].start()

        RSSI_DistroThread = Thread(target=RSSI_Handler.RSSI_Distributor,args = ())
        RSSI_DistroThread.setDaemon(True)
        RSSI_DistroThread.start()

        RSSI_AllocationThread = Thread(target=RSSI_Handler.RSSI_AllocationDistributor,args = ())
        RSSI_AllocationThread.setDaemon(True)
        RSSI_AllocationThread.start()

        tempThread = Thread(target=TemperatureHandler.TemperatureLoggerSocket, args = ())
        tempThread.setDaemon(True)
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
    
    # Safely end the ping logger thread 
    if PingLoggerThread.is_alive():
        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            GlobalVals.BREAK_PING_THREAD = True
        PingLoggerThread.join()
    
    # if this isn't the ground station 
    if not GlobalVals.IS_GROUND_STATION:
    
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
        
        # Safely end the imaginary balloon thread 
        if ImaginaryBalloonsThread.is_alive():
            with GlobalVals.BREAK_IMAGINARY_BALLOONS_MUTEX:
                GlobalVals.BREAK_IMAGINARY_BALLOONS_THREAD = True
            ImaginaryBalloonsThread.join()

        if EKF_Thread.is_alive():
            with GlobalVals.BREAK_EKF_LOGGER_THREAD_MUTEX:
                GlobalVals.BREAK_EKF_LOGGER_THREAD = True
            EKF_Thread.join()

        if EKF_DistroThread.is_alive():
            with GlobalVals.BREAK_EKF_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_EKF_DISTRO_THREAD = True
            EKF_Thread.join()

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
        