# import libraries 
import time
from threading import Thread

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter

#=====================================================
# Main function  
#=====================================================
def main():
    
    recievedPackets = False
    sendTime = int(time.time() + 1)

    # this loop will wait for packets and then process them 
    while True:

        curTime = int(time.time())
        
        # send dummy GPS data once a second  
        if curTime >= sendTime:
            
            sendTime = curTime + 1
            GPSpacket = CustMes.MESSAGE_FRAME()
            GPSdata = CustMes.MESSAGE_GPS()

            # set up GPS packet 
            GPSpacket.MessageID = 0x02
            GPSpacket.TargetID = 0
            GPSpacket.SystemID = GlobalVals.SYSTEM_ID

            # load dummy GPS data
            GPSdata.Longitude = 77.77
            GPSdata.Latitude = 88.88
            GPSdata.Altitude = 99.99
            GPSdata.GPSTime = 100.01

            # send dummy GPS packet
            GPSpacket.Payload = GPSdata.data_to_bytes()
            NetworkManager.sendPacket(GPSpacket)

        # check if packets have been recived 
        with GlobalVals.RECIEVED_PACKETS_MUTEX:
            if GlobalVals.RECIEVED_PACKETS:
                recievedPackets = True
        
        # if no packets have been recived then sleep and loop
        if not recievedPackets:
            continue
        else:
            recievedPackets = False
        
        # go through all the packets in the buffer 
        with GlobalVals.PACKET_BUFFER_IN_MUTEX:
            while len(GlobalVals.PACKET_BUFFER_IN) > 0:

                recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(0)

                # if the packet is a ping 
                if recievedPacket.MessageID == 1:
                    NetworkManager.PingRespond(recievedPacket.SystemID,recievedPacket.Timestamp)

                # if the packet is a GPS data packet 
                if recievedPacket.MessageID == 2:
                    
                    # get the GPS data  
                    recData = CustMes.MESSAGE_GPS()                 
                    error = recData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Dummy Radio 1: GPS data error " + str(error) + ".\n")
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
                        print ("Dummy Radio 1: Message error " + str(error) + ".\n")
                        continue
                        
                    # print string 
                    print(StrData.MessageStr)

                    continue


#=====================================================
# Thread starter 
#=====================================================
if __name__ == '__main__':

    GlobalVals.PORT = "COM6"
    GlobalVals.SYSTEM_ID = 2
    GlobalVals.ERROR_LOG_FILE = "ErrorLog2.txt"
    GlobalVals.PACKET_STATS_FILE = "PacketStats2.txt"

    # Start serial thread 
    NetworkThread = Thread(target=NetworkManager.SerialManagerThread,args=())
    NetworkThread.daemon = True
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.start()

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
    