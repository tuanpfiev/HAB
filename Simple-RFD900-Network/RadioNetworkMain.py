# import libraries 
import time
from threading import Thread
from copy import deepcopy
import sys
import gc

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter
import GPSHandler
import PingLogger
import ImaginaryBalloons

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
            print("main1")

            # go through all the packets in the buffer 
            while True:
                with GlobalVals.PACKET_BUFFER_IN_MUTEX:
                    if len(GlobalVals.PACKET_BUFFER_IN) > 0:
                        recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(0)
                    else:
                        break
                print("loop")
                # if the packet is a ping 
                if recievedPacket.MessageID == 1:
                    
                    # get the payload of the ping
                    pingPayload = CustMes.MESSAGE_PING()
                    error = pingPayload.bytes_to_data(recievedPacket.Payload)

                    # if there was an error with the payload report it 
                    if error != 0:
                        print("Ping Data Error: RadioNetworkMain:" + str(error))
                        print(recievedPacket.Payload.hex())
                        print(recievedPacket.SystemID)
                        print('\n')
                    
                    else:
                    
                        # check if the ping is an intiator 
                        if pingPayload.Intiator:
                            
                            # respond to the ping if it is  
                            NetworkManager.PingRespond(recievedPacket.SystemID,recievedPacket.Timestamp)
                        
                        else:

                            # send ping to ping thread if it isn't 
                            with GlobalVals.PACKET_PING_BUFFER_MUTEX:
                                GlobalVals.PACKET_PING_BUFFER.append(recievedPacket)

                            # set the recieved flag
                            with GlobalVals.RECIEVED_PING_MUTEX:
                                GlobalVals.RECIEVED_PING = True
                    

                # if the packet is a GPS data packet 
                if recievedPacket.MessageID == 2:
                    
                    # get the GPS data
                    GPSdata = CustMes.MESSAGE_GPS()                    
                    error = GPSdata.bytes_to_data(recievedPacket.Payload)

                    # report any errors
                    if error != 0:
                        print("GPS Data Error: RadioNetworkMain:" + str(error))
                        print(recievedPacket.Payload.hex())
                        print(recievedPacket.SystemID)
                        print('\n')
                    else:
                        # print GPS data to screen 
                        print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                        print("Lon:" + str(GPSdata.Longitude) + ", Lat:" + str(GPSdata.Latitude) + ", Alt:" + str(GPSdata.Altitude) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                        # set the system id for the GPS data
                        GPSdata.SystemID = recievedPacket.SystemID
        
                        # put data into the buffer
                        with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                            GlobalVals.GPS_DATA_BUFFER.append(GPSdata)

                        # set the flags for the buffer 
                        with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
                            GlobalVals.RECIEVED_GPS_RADIO_DATA = True
                    
                
                # if the packet is string message  
                if recievedPacket.MessageID == 3: 

                    # get the string  
                    StrData = CustMes.MESSAGE_STR()                    
                    error = StrData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Packet error, packet will be discarded.\n")
                    else:
                        # print string 
                        print(StrData.MessageStr)
                
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

    # get arguments for running the script

    
    # check the type is correct
    # if sys.argv[1] == 'g':
    #     GlobalVals.IS_GROUND_STATION = True
    # elif sys.argv[1] == 'b':
    GlobalVals.IS_GROUND_STATION = False
    # else:
        # print("ERROR: Arg1 isn't recognised. Closing program.") 
        # sys.exit() 
    
    # check the system ID is in range
    sysID = 1

    # set system ID and serial port
    GlobalVals.SYSTEM_ID = sysID
    GlobalVals.PORT = "/dev/ttyUSB0"

    # setup the log files 
    GlobalVals.ERROR_LOG_FILE = "ErrorLog_" + str(sysID) + ".txt"
    GlobalVals.PING_LOG_FILE = "PingLog_" + str(sysID) + ".txt"
    GlobalVals.PACKET_STATS_FILE = "PacketStats_" + str(sysID) + ".txt"
    GlobalVals.GROUND_STATION_LOG_FILE = "GSLog_" + str(sysID) + ".txt"

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
        # # Start GPS distributor 
        # GPSDistroThread = Thread(target=GPSHandler.GPSDistributor, args=())
        # GPSDistroThread.setDaemon(True)
        # GPSDistroThread.start()

        # Start imaginary balloon socket
        ImaginaryBalloonsThread = Thread(target=ImaginaryBalloons.ImaginaryBalloons, args=())
        ImaginaryBalloonsThread.setDaemon(True)
        ImaginaryBalloonsThread.start()

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
        # if GPSDistroThread.is_alive():
        #     with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
        #         GlobalVals.BREAK_GPS_DISTRO_THREAD = True
        #     GPSDistroThread.join()
        
        # Safely end the imaginary balloon thread 
        if ImaginaryBalloonsThread.is_alive():
            with GlobalVals.BREAK_IMAGINARY_BALLOONS_MUTEX:
                GlobalVals.BREAK_IMAGINARY_BALLOONS_THREAD = True
            ImaginaryBalloonsThread.join()
    