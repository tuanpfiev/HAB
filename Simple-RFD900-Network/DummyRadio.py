# import libraries 
import time
from threading import Thread
import sys

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter
import PingLogger

SEND_INTERVAL = 2

#=====================================================
# Main function  
#=====================================================
def main():

    global SEND_INTERVAL
    
    recievedPackets = False
    sendTime = int(time.time() + SEND_INTERVAL)

    # this loop will wait for packets and then process them 
    while True:

        curTime = int(time.time())
        
        # send dummy GPS data once a second  
        if curTime >= sendTime:
            
            sendTime = curTime + SEND_INTERVAL
            GPSpacket = CustMes.MESSAGE_FRAME()
            GPSdata = CustMes.MESSAGE_GPS()

            # set up GPS packet 
            GPSpacket.MessageID = 0x02
            GPSpacket.TargetID = CustMes.BROADCAST_ID
            GPSpacket.SystemID = GlobalVals.SYSTEM_ID

            # load dummy GPS data
            GPSdata.Longitude = GlobalVals.SYSTEM_ID * 10.10
            GPSdata.Latitude = GlobalVals.SYSTEM_ID * 11.11
            GPSdata.Altitude = GlobalVals.SYSTEM_ID * 12.12
            GPSdata.GPSTime = GlobalVals.SYSTEM_ID * 13.13

            # send dummy GPS packet
            GPSpacket.Payload = GPSdata.data_to_bytes()
            NetworkManager.sendPacket(GPSpacket)

        # check if packets have been recived 
        with GlobalVals.RECIEVED_PACKETS_MUTEX:
            if GlobalVals.RECIEVED_PACKETS:
                recievedPackets = True
        
        # if no packets have been recived then loop
        if not recievedPackets:
            continue
        else:
            recievedPackets = False
        
        # go through all the packets in the buffer 
        while True:
            with GlobalVals.PACKET_BUFFER_IN_MUTEX:
                if len(GlobalVals.PACKET_BUFFER_IN) > 0:
                    recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(0)
                else:
                    break

            # if the packet is a ping 
            if recievedPacket.MessageID == 1:
                
                # get the payload of the ping
                pingPayload = CustMes.MESSAGE_PING()
                error = pingPayload.bytes_to_data(recievedPacket.Payload)

                # if there was an error with the payload report it 
                if error != 0:
                    print("Ping Data Error: RadioNetworkMain:" + str(error))
                    print(recievedPacket.Payload)
                    print(recievedPacket.SystemID)
                    print('\n')
                    continue
                
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
                
                continue

            # if the packet is a GPS data packet 
            if recievedPacket.MessageID == 2:
                
                # get the GPS data  
                recData = CustMes.MESSAGE_GPS()                 
                error = recData.bytes_to_data(recievedPacket.Payload)
                if error != 0:
                    print ("Dummy Radio: GPS data error: " + str(error) + ".\n")
                    continue
                
                print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                print("Lon:" + str(recData.Longitude) + ", Lat:" + str(recData.Latitude) + ", Alt:" + str(recData.Altitude) + ", Time:" + str(recData.GPSTime) + "\n") 
                
                continue
            
            # if the packet is string message  
            if recievedPacket.MessageID == 3: 

                # get the string  
                StrData = CustMes.MESSAGE_STR()                    
                error = StrData.bytes_to_data(recievedPacket.Payload)
                if error != 0:
                    print ("Dummy Radio: Message error: " + str(error) + ".\n")
                    continue
                    
                # print string 
                print(StrData.MessageStr)

                continue


#=====================================================
# Thread starter 
#=====================================================
if __name__ == '__main__':

    GlobalVals.IS_GROUND_STATION = False

    # get arguments for running the script
    numArgs = len(sys.argv)
    
    # check if the number of args are correct 
    if numArgs != 3:
        print("Incorrect number of Args.")
        print(numArgs)
        sys.exit() 
    
    # check the system ID is in range
    sysID = int(sys.argv[1])
    if sysID < 1 or sysID > 255:
        print("ERROR: Arg1 isn't in range (1 - 255). Closing program.")
        sys.exit()

    # set system ID and serial port
    GlobalVals.SYSTEM_ID = sysID
    GlobalVals.PORT = sys.argv[2]

    # setup the log files 
    GlobalVals.ERROR_LOG_FILE = "ErrorLog_" + sys.argv[1] + ".txt"
    GlobalVals.PING_LOG_FILE = "PingLog_" + sys.argv[1] + ".txt"
    GlobalVals.PACKET_STATS_FILE = "PacketStats_" + sys.argv[1] + ".txt"

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