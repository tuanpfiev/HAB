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
import sys

sys.path.insert(1,'../utils')
from utils import get_port
from navpy import lla2ecef
import numpy as np

global GPS_log
GPS_log = np.array(([1.0, 1.0, 1.0],[2.0, 2.0, 2.0]))


def update_GPS_log(gps_data):
    global GPS_log
    index = gps_data.SystemID
    
    GPS_log[index-1,:]=[gps_data.Latitude, gps_data.Longitude, gps_data.Altitude]

def distance_calculation(gps_data):
    p1 = lla2ecef(gps_data[0,0],gps_data[0,1],gps_data[0,2])
    p2 = lla2ecef(gps_data[1,0],gps_data[1,1],gps_data[1,2])
    return np.linalg.norm(p1-p2)


#=====================================================
# Main function  
#=====================================================
def main():
    
    recievedPackets = False
    # sendTime = int(time.time() + 1)

    # this loop will wait for packets and then process them 
    while True:

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
                    
                    print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                    print("Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,2)) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # set the system id for the GPS data
                    GPSdata.SystemID = recievedPacket.SystemID

                    # update GPS_log
                    update_GPS_log(GPSdata)
                    distance = distance_calculation(GPS_log)
                    print('Distance: ',round(distance,2)," [m]")

                    # put data into the buffer
                    with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                        GlobalVals.GPS_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_GPS_RADIO_DATA = True
                    
                    continue

                
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
                print("Exception: " + str(e.__class__))
                print(e)
                print("Error using GPS data log file")


#=====================================================
# Thread starter 
#=====================================================
if __name__ == '__main__':

    # set Port
    GlobalVals.PORT=get_port('RFD900')
    print('PORT: '+ GlobalVals.PORT)

    # Start serial thread 
    NetworkThread = Thread(target=NetworkManager.SerialManagerThread,args=())
    NetworkThread.daemon = True
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.start()

    # Start logger socket
    # GPSLoggerThread = Thread(target=GPSHandler.GPSLoggerSocket, args=())
    # GPSLoggerThread.start()

    # Start GPS distributor 
    # GPSDistroThread = Thread(target=GPSHandler.GPSDistributor, args=())
    # GPSDistroThread.start()

    # Start Ping Logger 
    PingLoggerThread = Thread(target=PingLogger.PingLoggerThread, args=())
    PingLoggerThread.start()

    # Start imaginary balloon socket
    # ImaginaryBalloonsThread = Thread(target=ImaginaryBalloons.ImaginaryBalloons, args=())
    # ImaginaryBalloonsThread.start()

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
    # if GPSLoggerThread.is_alive():
    #     with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
    #         GlobalVals.BREAK_GPS_LOGGER_THREAD = True
    #     GPSLoggerThread.join()
    
    # Safely end the GPS distributor thread 
    # if GPSDistroThread.is_alive():
    #     with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
    #         GlobalVals.BREAK_GPS_DISTRO_THREAD = True
    #     GPSDistroThread.join()
    
    # Safely end the ping logger thread 
    if PingLoggerThread.is_alive():
        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            GlobalVals.BREAK_PING_THREAD = True
        PingLoggerThread.join()
    
    # if ImaginaryBalloonsThread.is_alive():
    #     with GlobalVals.BREAK_IMAGINARY_BALLOONS_MUTEX:
    #         GlobalVals.BREAK_IMAGINARY_BALLOONS_THREAD = True
    #     ImaginaryBalloonsThread.join()
    
