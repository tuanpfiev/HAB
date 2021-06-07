# import libraries 
import time
from threading import Thread
import calendar
from datetime import datetime

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter
import GPSHandler
import PingLogger
import ImaginaryBalloons
import EKFHandler
import sys, os
import copy

sys.path.insert(1,'../utils')
from utils import get_port
from navpy import lla2ecef
import numpy as np
import json
from common import *
from common_class import *
import random
import boto3
from threading import Lock
import csv
import math

GPS_Log_LOCK = Lock()
EKF_LOG_LOCK = Lock()
CUSTMES_LOCK = Lock()

class Path:

    def __init__(self, filePath = None, sysID = None):

        self.filePath = filePath if filePath is not None else ""
        self.sysID = sysID if sysID is not None else 1
        self.trajectory = []
        
        with open(self.filePath) as csv_file:
            
            self.reader = csv.reader(csv_file, delimiter=',')
            
            for row in self.reader:
                gps = GPS(self.sysID,float(row[1]),float(row[2]),float(row[3]), float(row[0]))
                self.trajectory.append(gps)

        self.epoch_range = [self.trajectory[0].epoch, self.trajectory[-1].epoch]

    def getGPS(self, query_epoch):

        if query_epoch <= self.epoch_range[0]:
            return self.trajectory[0]

        if query_epoch >= self.epoch_range[-1]:
            return self.trajectory[-1]

        for i in range(len(self.trajectory)):
            if self.trajectory[i].epoch > query_epoch:
                idx = i-1
                break

        ratio = (query_epoch - self.trajectory[idx].epoch) / (self.trajectory[idx+1].epoch - self.trajectory[idx].epoch)

        lat_interp = self.trajectory[idx].lat + (self.trajectory[idx+1].lat - self.trajectory[idx].lat) * ratio
        lon_interp = self.trajectory[idx].lon + (self.trajectory[idx+1].lon - self.trajectory[idx].lon) * ratio
        alt_interp = self.trajectory[idx].lat + (self.trajectory[idx+1].alt - self.trajectory[idx].alt) * ratio

        return GPS(self.sysID,lat_interp,lon_interp,alt_interp,query_epoch)
    
    def getDistance(self, gps):

        predictedGPS = self.getGPS(gps.epoch)

        positionENU_Relative = positionENU(gps,predictedGPS)
        distance = math.sqrt(positionENU_Relative[0]**2+positionENU_Relative[1]**2)
        
        return distance

def distanceMatrixCalculation(gpsAll):

    distanceMatrix = np.zeros([GlobalVals.N_REAL_BALLOON,GlobalVals.N_REAL_BALLOON])

    for i in range(GlobalVals.N_REAL_BALLOON):
        for j in range(i+1,GlobalVals.N_REAL_BALLOON):
            distanceMatrix[i][j] = distance_calculation(gpsAll[i],gpsAll[j])
            distanceMatrix[j][i] = distanceMatrix[i][j]

    return distanceMatrix

def distanceMatrixCalculation(gpsAll,nRound):

    distanceMatrix = np.zeros([GlobalVals.N_REAL_BALLOON,GlobalVals.N_REAL_BALLOON])

    for i in range(GlobalVals.N_REAL_BALLOON):
        for j in range(i+1,GlobalVals.N_REAL_BALLOON):
            distanceMatrix[i][j] = round(distance_calculation(gpsAll[i],gpsAll[j]),nRound)
            distanceMatrix[j][i] = distanceMatrix[i][j]

    return distanceMatrix

def distanceEKF_MatrixCalculation(gpsAll,nRound):

    distanceMatrix = np.zeros([GlobalVals.N_REAL_BALLOON,GlobalVals.N_REAL_BALLOON])

    for i in range(GlobalVals.N_REAL_BALLOON):
        for j in range(i+1,GlobalVals.N_REAL_BALLOON):
            distanceMatrix[i][j] = round(distance_calculation(gpsAll[i],gpsAll[j]),nRound)
            distanceMatrix[j][i] = distanceMatrix[i][j]

    return distanceMatrix


def dataTruncate(dataList,nData,nBalloon):
    if len(dataList)==0 or len(dataList)<nData*nBalloon:
        return dataList
    
    # interval = int(len(dataList)/nData/nBalloon)
    interval = nData*nBalloon

    selectedIndex = np.array([],dtype=np.int64)
    for i in range(nBalloon):
        tmp = np.arange(i,len(dataList),interval)
        selectedIndex = np.append(selectedIndex,tmp)

    selectedIndex = np.sort(selectedIndex)
    result = []
    for i in range(len(selectedIndex)):
        result.append(dataList[selectedIndex[i]])
    
    # add the last element:
    for i in range(nBalloon):
        result.append(dataList[-1-i])
    return result


# global balloonPaths
balloonPaths = []
baloonPathAll = [Path("Balloon_254.csv", 1), Path("Balloon_253.csv", 2), Path("Balloon_255.csv", 3)]
baloonPathAllOriginal = baloonPathAll[:]
nTruncate = 20
fireLocation = GPS(None,-37.62351737207409, 145.12652094970966, 0);


for i in range(len(baloonPathAll)):
    baloonPathAll[i].trajectory = dataTruncate(baloonPathAll[i].trajectory,nTruncate,1)

for i in range(len(baloonPathAll)):
    for j in range(len(baloonPathAll[0].trajectory)):
        each_balloon = {
                    'idP': str(baloonPathAll[i].trajectory[j].sysID),
                    'tP': str(baloonPathAll[i].trajectory[j].epoch),
                    'latP': str(baloonPathAll[i].trajectory[j].lat),
                    'lonP': str(baloonPathAll[i].trajectory[j].lon),
                    'altP': str(baloonPathAll[i].trajectory[j].alt)
                }
        balloonPaths.append(each_balloon)



global  count_history,pathHistory,count_t
count_history = 1
pathHistory = []
count_t = 0

def update_temperature(tempur):
    index = tempur.sysID
    GlobalVals.TEMPERATURE_ALL[index-1]=tempur

def update_GPS_Log(gps_data):
    index = gps_data.SystemID
    try:    
        GlobalVals.GPS_ALL[index-1]= GPS(gps_data.SystemID, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here')

def update_EKF_Log(EKF_Data):
    index = EKF_Data.SystemID
    try:    
        GlobalVals.EKF_ALL[index-1]= EKF(EKF_Data.SystemID, EKF_Data.Latitude, EKF_Data.Longitude, EKF_Data.Altitude,EKF_Data.Epoch)
    except:
        print('here')


def distance_calculation(gps_data):
    p1 = lla2ecef(gps_data[0].lat,gps_data[0].lon,gps_data[0].alt)
    p2 = lla2ecef(gps_data[1].lat,gps_data[1].lon,gps_data[1].alt)
    return np.linalg.norm(p1-p2)

def distance_calculation(gps1,gps2):
    p1 = lla2ecef(gps1.lat,gps1.lon,gps1.alt)
    p2 = lla2ecef(gps2.lat,gps2.lon,gps2.alt)
    return np.linalg.norm(p1-p2)

def gps_lambda_handler():
    global count_t, count_history, pathHistory

    stream_name = 'RMITballoon_Data'
    k_client = boto3.client('kinesis', region_name='ap-southeast-2')
    nRealBalloon = GlobalVals.N_REAL_BALLOON

    time.sleep(5)
    while True:

        with GlobalVals.GPS_LOG_MUTEX:
            GPS_Log = copy.deepcopy(GlobalVals.GPS_ALL)

        with GlobalVals.TEMPERATURE_UPDATE_MUTEX:
            tempAll = copy.deepcopy(GlobalVals.TEMPERATURE_ALL)

        count_t = count_t + 1
        aws_message = balloonPaths[:]
        t0 = time.time()

        for i in range(len(GPS_Log)):
            if i < nRealBalloon:
                predictedOffset = baloonPathAllOriginal[i].getDistance(GPS_Log[i])
                targetOffset = baloonPathAllOriginal[i].getDistance(fireLocation)
                temperature = tempAll[i].temperature
            else:
                positionENU_RelativeEKF = positionENU(GPS_Log[i],GPS_Log[i-nRealBalloon])
                predictedOffset = math.sqrt(positionENU_RelativeEKF[0]**2+positionENU_RelativeEKF[1]**2)
                targetOffset = 0
                temperature = 0


            each_balloon = {
                'id': str(GPS_Log[i].sysID),
                't': str(t0),
                'lat': str(GPS_Log[i].lat ),
                'lon': str(GPS_Log[i].lon),
                'alt': str(GPS_Log[i].alt),
                'prs': str(0+ random.uniform(0,10)),
                'ss': str(0+ random.uniform(0,10)),
                'd': str(predictedOffset),
                'tar': str(targetOffset),
                'tmp': str(temperature)
            }
            aws_message.append(each_balloon)


        count_history = count_history + 1

        # if count_history % 5 == 0:
        for i in range(len(GPS_Log)):
            each_balloon = {
                'idH': str(GPS_Log[i].sysID),
                'tH': str(t0),
                'latH': str(GPS_Log[i].lat),
                'lonH': str(GPS_Log[i].lon),
                'altH': str(random.uniform(200,300)),
                'tmpH': str(temperature)
            }
            pathHistory.append(each_balloon)

        pathHistory = dataTruncate(pathHistory,nTruncate,len(GPS_Log))

        for i in pathHistory:
            aws_message.append(i)
        
        initial_message = {
            'nBalloon': str(nRealBalloon),  
            'launch': "Horsham",
            'nFire': str(2),
            'fireLocation': str([-37.62351737207409, 145.12652094970966,-37.62351737207409, 145.12652094970966])
        }
        aws_message.append(initial_message)

        # response = k_client.put_record(
        #         StreamName=stream_name,
        #         Data=json.dumps(aws_message),
        #         PartitionKey=str(random.randrange(10000))
        # )
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        # print(json.dumps(aws_message))
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        print("Publishing to AWS Kinesis Data ...")
        time.sleep(3)
        
# def gps_lambda_handler():
#     global count_t, count_history, pathHistory

#     stream_name = 'RMITballoon_Data'
#     k_client = boto3.client('kinesis', region_name='ap-southeast-2')

#     while True:
                    
#         # print("***************************************************************************")
#         # print(GlobalVals.AWS_GPS_DATA_BUFFER)
#         # print("***************************************************************************")

#         with GPS_Log_LOCK:
#             with EKF_LOG_LOCK:
#                 GPS_Log_tmp = GPS_Log

#         aws_message = balloonPaths[:]
#         t0 = time.time()

#         for i in range(len(GPS_Log_tmp)):
#             if i < nRealBalloon:
#                 predictedOffset = baloonPathAllOriginal[i].getDistance(GPS_Log_tmp[i])
#                 targetOffset = baloonPathAllOriginal[i].getDistance(fireLocation)
#             else:
#                 positionENU_RelativeEKF = positionENU(GPS_Log_tmp[i],GPS_Log_tmp[i-nRealBalloon])
#                 predictedOffset = math.sqrt(positionENU_RelativeEKF[0]**2+positionENU_RelativeEKF[1]**2)
#                 targetOffset = 0

#             temperatureOutside = random.uniform(50,60)

#             each_balloon = {
#                 'id': str(GPS_Log_tmp[i].sysID),
#                 't': str(t0),
#                 'lat': str(GPS_Log_tmp[i].lat ),
#                 'lon': str(GPS_Log_tmp[i].lon),
#                 'alt': str(GPS_Log_tmp[i].alt),
#                 'prs': str(0),
#                 'ss': str(0),
#                 'd': str(predictedOffset),
#                 'tar': str(targetOffset),
#                 'tmp': str(temperatureOutside)
#             }
#             # print(targetOffset)
#             aws_message.append(each_balloon)
    
#         for i in range(len(GPS_Log_tmp)):
#             each_balloon = {
#                 'idH': str(GPS_Log_tmp[i].sysID),
#                 'tH': str(t0),
#                 'latH': str(GPS_Log_tmp[i].lat),
#                 'lonH': str(GPS_Log_tmp[i].lon),
#                 'altH': str(GPS_Log_tmp[i].alt),
#                 'tmpH': str(temperatureOutside)
#             }
#             pathHistory.append(each_balloon)

#         pathHistory = dataTruncate(pathHistory,nTruncate,len(GPS_Log_tmp))

#         for i in pathHistory:
#             aws_message.append(i)

#         response = k_client.put_record(
#                 StreamName=stream_name,
#                 Data=json.dumps(aws_message),
#                 PartitionKey=str(random.randrange(10000))
#         )
#         #print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
#         #print(json.dumps(aws_message))
#         #print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")

#         time.sleep(3)
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

                    if not GPSHandler.GPS_FormatCheck(GPSdata):
                        print("GPS message via RFD900 was broken. Discard it...") 
                        continue

                    print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                    print("Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,2)) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # update GPS_Log
                    with GlobalVals.GPS_LOG_MUTEX:
                        update_GPS_Log(GPSdata)
                        distance = distanceMatrixCalculation(GlobalVals.GPS_ALL,0)
                    print("--------------------------------------------------------------------------------------------------")
                    print("GPS GPS GPS " + str(recievedPacket.SystemID) + str(recievedPacket.SystemID) + str(recievedPacket.SystemID) + ":" + " Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,1)) + ", Time:" + str(round(GPSdata.GPSTime,1)))
                    print('Distance from GPS [m]:\n',distance)

                    # put data into the buffer
                    # with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                    #     GlobalVals.GPS_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    # with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
                    #     GlobalVals.RECIEVED_GPS_RADIO_DATA = True
                    
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
                    EKF_Data = CustMes.MESSAGE_EKF()                    
                    error = EKF_Data.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: EKF data error " + str(error) + ".\n")
                        continue

                    # set the system id for the GPS data
                    EKF_Data.SystemID = recievedPacket.SystemID
                    
                    if not EKFHandler.EKF_FormatCheck(EKF_Data):
                        print("EKF message via RFD900 was broken. Discard it...") 
                        continue

                    # update GPS_Log
                    with GlobalVals.EKF_LOG_MUTEX:
                        update_EKF_Log(EKF_Data)
                        distance = distanceEKF_MatrixCalculation(GlobalVals.EKF_ALL,0,)

                    print("==================================================================================================")
                    print("EKF EKF EKF " + str(recievedPacket.SystemID) +str(recievedPacket.SystemID) +str(recievedPacket.SystemID) + ":" + " Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,1)) + ", Time:" + str(round(GPSdata.GPSTime,1)))
                    print('Distance from EKF [m]:\n',distance)

                    continue

                # if the packet is an temperature data packet 
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
                    
                    if not TemperatureHandler.temperatureFormatCheck(temperatureData):
                        print("Temperature message via RFD900 was broken. Discard it...")
                        continue
                    
                    tempur = TEMPERATURE(temperatureData.SystemID,temperatureData.Temperature,temperatureData.Epoch)
                    with GlobalVals.TEMPERATURE_UPDATE_MUTEX:
                        update_temperature(tempur)
                    # print(" Temperature Data from " + str(recievedPacket.SystemID) + ":" + "Temperature:" + str(round(temperatureData.Temperature,1)))



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

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass
    

    GlobalVals.ERROR_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ErrorLog.txt"
    GlobalVals.PING_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-PingLog.txt"
    GlobalVals.PACKET_STATS_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-PacketStats.txt"
    GlobalVals.GROUND_STATION_LOG_FILE = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-GPS_GroundStationLogger.txt"

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
    #     with GlobalVals.BREAK_GPS_LogGER_THREAD_MUTEX:
    #         GlobalVals.BREAK_GPS_LogGER_THREAD = True
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
