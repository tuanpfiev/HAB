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
import sys, os

sys.path.insert(1,'../utils')
from utils import get_port
from navpy import lla2ecef
import numpy as np
import json
from common import *
from common_class import *
import random
import boto3



global GPS_log, count_history
n_real_balloon = 2
GPS_log = np.array([GPS()]*n_real_balloon)
count_history = np.array([0]*n_real_balloon)

def update_GPS_log(gps_data):
    global GPS_log
    index = gps_data.SystemID
    try:    
        GPS_log[index-1]= GPS(gps_data.SystemID, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here')

def distance_calculation(gps_data):
    p1 = lla2ecef(gps_data[0].lat,gps_data[0].lon,gps_data[0].alt)
    p2 = lla2ecef(gps_data[1].lat,gps_data[1].lon,gps_data[1].alt)
    return np.linalg.norm(p1-p2)

def gps_lambda_handler():

    stream_name = 'RMITballoon_Data'
    k_client = boto3.client('kinesis', region_name='ap-southeast-2')

    while True:
                    
        # print("***************************************************************************")
        # print(GlobalVals.AWS_GPS_DATA_BUFFER)
        # print("***************************************************************************")
        aws_message = []
        for i in range(len(GPS_log)):
            each_balloon = {
                'sysID': str(1),
                'timeStamp': str(GPS_log[i].epoch),
                'lat': str(GPS_log[i].lat),
                'lon': str(GPS_log[i].lon),
                'alt': str(GPS_log[i].alt),
                'pressure': str(0),
                'signal_strength': str(0)
            }
            aws_message.append(each_balloon)

        path = []
        for i in range(len(GPS_log)):
            each_balloon = {
                'sysID_h': str(1),
                'time_h': str(GPS_log[i].epoch),
                'lat_h': str(GPS_log[i].lat),
                'lon_h': str(GPS_log[i].lon),
                'alt_h': str(GPS_log[i].alt),

            }
            path.append(each_balloon)
        
        aws_message.append(path)

        response = k_client.put_record(
                StreamName=stream_name,
                Data=json.dumps(aws_message),
                # Data = '[{"sysID": "0", "timeStamp": "1616664193158.4873", "lat": "5.206728581869298", "lon": "32.464139215264524", "alt": "200.19366866684368", "pressure": "1538.358674366736", "signal_strength": "-78.84911771441294"}, {"sysID": "1", "timeStamp": "1616664193.1584873", "lat": "5.308554001275203", "lon": "32.37461053504348", "alt": "227.88151976770965", "pressure": "1661.610419273367", "signal_strength": "-76.36356412110052"}, {"sysID": "2", "timeStamp": "1616664193.1584873", "lat": "8.123481650386552", "lon": "37.87815098798116", "alt": "278.68562668245113", "pressure": "1353.3783098051533", "signal_strength": "-89.91233630179218"}, {"sysID": "3", "timeStamp": "1616664193.1584873", "lat": "6.489141170725179", "lon": "33.53937285828535", "alt": "224.23065648851028", "pressure": "1538.9333055493116", "signal_strength": "-78.84729804332947"}, {"sysID": "4", "timeStamp": "1616664193.1584873", "lat": "1.2494047201518554", "lon": "39.538746616103836", "alt": "227.66821532645596", "pressure": "1757.8155350482111", "signal_strength": "-89.95481461416858"}, [{"sysID_h": "0", "time_h": "1616664193.1584873", "lat_h": "-36.71756565615016", "lon_h": "142.21421971321564", "alt_h": "297.2000452618081"}, {"sysID_h": "1", "time_h": "1616664193.1584873", "lat_h": "-36.71907273180851", "lon_h": "142.20937797351252", "alt_h": "243.4553806587891"}, {"sysID_h": "2", "time_h": "1616664193.1584873", "lat_h": "-36.71538691546569", "lon_h": "142.18636041981281", "alt_h": "210.70848360023982"}, {"sysID_h": "3", "time_h": "1616664193.1584873", "lat_h": "-36.715342802396464", "lon_h": "142.2777564240451", "alt_h": "280.72262902738765"}, {"sysID_h": "4", "time_h": "1616664193.1584873", "lat_h": "-36.718430732855374", "lon_h": "142.11824349751765", "alt_h": "270.1717911308283"}, {"sysID_h": "0", "time_h": "1616664193.1584873", "lat_h": "-36.71637743399295", "lon_h": "142.14831562184483", "alt_h": "213.859304696758"}, {"sysID_h": "1", "time_h": "1616664193.1584873", "lat_h": "-36.71980524861051", "lon_h": "142.12980132216143", "alt_h": "254.33563542394694"}, {"sysID_h": "2", "time_h": "1616664193.1584873", "lat_h": "-36.71934078447395", "lon_h": "142.219660100136", "alt_h": "239.80918393657032"}, {"sysID_h": "3", "time_h": "1616664193.1584873", "lat_h": "-36.7153725217811", "lon_h": "142.16116885498164", "alt_h": "284.34822781414795"}, {"sysID_h": "4", "time_h": "1616664193.1584873", "lat_h": "-36.7189435667627", "lon_h": "142.10132189614723", "alt_h": "296.39948010459364"}]]',
                PartitionKey=str(random.randrange(10000))
        )
        print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        print(json.dumps(aws_message))
        print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")

        time.sleep(1)
        

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

                # if the packet is an EKF GPS data packet 
                if recievedPacket.MessageID == 5:
                    
                    # get the GPS data
                    GPSdata = CustMes.MESSAGE_GPS()                    
                    error = GPSdata.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: GPS data error " + str(error) + ".\n")
                        continue
                    print("=================================================")
                    print("EKF GPS Data from " + str(recievedPacket.SystemID) + ":")
                    print("Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,2)) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # set the system id for the GPS data
                    GPSdata.SystemID = recievedPacket.SystemID

                    # update GPS_log
                    update_GPS_log(GPSdata)
                    distance = distance_calculation(GPS_log)
                    print('Distance EKF: ',round(distance,2)," [m]")
                    print('------------------------------------------------')
                    # put data into the buffer
                    with GlobalVals.EKF_GPS_DATA_BUFFER_MUTEX:
                        GlobalVals.EKF_GPS_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    # with GlobalVals.RECIEVED_EKF_GPS_RADIO_DATA_MUTEX:
                    #     GlobalVals.RECIEVED_EKF_GPS_RADIO_DATA = True
                    
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

                    # AWS msg
                    # each_balloon = {
                    #     'sysID': str(SystemID),
                    #     'timeStamp': str(GPSData.GPSTime),
                    #     'lat': str(Latitude),
                    #     'lon': str(Longitude),
                    #     'alt': str(Altitude)
                    # }

                    # with GlobalVals.AWS_GPS_DATA_BUFFER_MUTEX:
                    #     GlobalVals.AWS_GPS_DATA_BUFFER.append(each_balloon)

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

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

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

    # Start AWS thread
    AWS_GPS_Thread = Thread(target = gps_lambda_handler, args=())
    AWS_GPS_Thread.start()

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
    
    if AWS_GPS_Thread.is_alive():
        AWS_GPS_Thread.join()
