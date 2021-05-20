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
baloonPathAll = [Path("Balloon_254.csv", 1), Path("Balloon_253.csv", 2)]
baloonPathAllOriginal = baloonPathAll[:]
nTruncate = 50
fireLocation = GPS(None,-36.373326870216395, 142.36570090762967, 0);


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


global GPS_Log, nRealBalloon, count_history,pathHistory,count_t
nRealBalloon = 2
# GPS_Log = np.array([GPS()]*nRealBalloon)
GPS_Log = np.array([GPS(1), GPS(2),GPS(3), GPS(4)])
count_history = 1
pathHistory = []
count_t = 0


def update_GPS_Log(gps_data):
    global GPS_Log
    index = gps_data.SystemID
    try:    
        GPS_Log[index-1]= GPS(gps_data.SystemID, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here')

def update_GPS_EKF_log(gps_data):
    global GPS_Log, nRealBalloon
    index = gps_data.SystemID
    try:    
        GPS_Log[index-1+nRealBalloon]= GPS(gps_data.SystemID+nRealBalloon, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here')


def distance_calculation(gps_data):
    p1 = lla2ecef(gps_data[0].lat,gps_data[0].lon,gps_data[0].alt)
    p2 = lla2ecef(gps_data[1].lat,gps_data[1].lon,gps_data[1].alt)
    return np.linalg.norm(p1-p2)

def gps_lambda_handler():
    # global balloonPaths
    global count_t, count_history, pathHistory

    stream_name = 'RMITballoon_Data'
    k_client = boto3.client('kinesis', region_name='ap-southeast-2')

    np.random.seed(1)
    init_lat = np.random.uniform(low=-36.78, high = -36.8,size=(len(GPS_Log),1))
    np.random.seed(1)
    init_lon = np.random.uniform(low=142.1, high = 142.9,size=(len(GPS_Log),1))

    vlat = np.array([0.01, 0.02,0.024, 0.03])*0.2
    while True:
        count_t = count_t + 1
        aws_message = balloonPaths[:]
        t0 = time.time()

        for i in range(len(GPS_Log)):
            if i < nRealBalloon:
                predictedOffset = baloonPathAllOriginal[i].getDistance(GPS_Log[i])
                targetOffset = baloonPathAllOriginal[i].getDistance(fireLocation)
            else:
                positionENU_RelativeEKF = positionENU(GPS_Log[i],GPS_Log[i-nRealBalloon])
                predictedOffset = math.sqrt(positionENU_RelativeEKF[0]**2+positionENU_RelativeEKF[1]**2)
                targetOffset = 0

            temperatureOutside = random.uniform(50,60)

            GPS_Log[i].lat = init_lat[i][0]+ count_t * vlat[i]
            GPS_Log[i].lon = init_lon[i][0]+ count_t * 0.01
            GPS_Log[i].alt = count_t * vlat[i]*100
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
                'tmp': str(temperatureOutside)
            }
            aws_message.append(each_balloon)

        path = []
        count = 0
        count_history = count_history + 1

        # if count_history % 5 == 0:
        for i in range(len(GPS_Log)):
            each_balloon = {
                'idH': str(GPS_Log[i].sysID),
                'tH': str(t0),
                'latH': str(GPS_Log[i].lat),
                'lonH': str(GPS_Log[i].lon),
                'altH': str(random.uniform(200,300)),
                'tmpH': str(temperatureOutside)
            }
            pathHistory.append(each_balloon)

        pathHistory = dataTruncate(pathHistory,nTruncate,len(GPS_Log))

        for i in pathHistory:
            aws_message.append(i)
        
        initial_message = {
            'nBalloon': str(nRealBalloon),  
            'launch': "Horsham",
            'nFire': str(2),
            'fireLocation': str([-36.7, 142.2, -36.8, 142.4])
        }
        aws_message.append(initial_message)

        response = k_client.put_record(
                StreamName=stream_name,
                Data=json.dumps(aws_message),
                # Data = '[{"sysID": "0", "timeStamp": "1616664193158.4873", "lat": "5.206728581869298", "lon": "32.464139215264524", "alt": "200.19366866684368", "pressure": "1538.358674366736", "signal_strength": "-78.84911771441294"}, {"sysID": "1", "timeStamp": "1616664193.1584873", "lat": "5.308554001275203", "lon": "32.37461053504348", "alt": "227.88151976770965", "pressure": "1661.610419273367", "signal_strength": "-76.36356412110052"}, {"sysID": "2", "timeStamp": "1616664193.1584873", "lat": "8.123481650386552", "lon": "37.87815098798116", "alt": "278.68562668245113", "pressure": "1353.3783098051533", "signal_strength": "-89.91233630179218"}, {"sysID": "3", "timeStamp": "1616664193.1584873", "lat": "6.489141170725179", "lon": "33.53937285828535", "alt": "224.23065648851028", "pressure": "1538.9333055493116", "signal_strength": "-78.84729804332947"}, {"sysID": "4", "timeStamp": "1616664193.1584873", "lat": "1.2494047201518554", "lon": "39.538746616103836", "alt": "227.66821532645596", "pressure": "1757.8155350482111", "signal_strength": "-89.95481461416858"}, [{"sysID_h": "0", "time_h": "1616664193.1584873", "lat_h": "-36.71756565615016", "lon_h": "142.21421971321564", "alt_h": "297.2000452618081"}, {"sysID_h": "1", "time_h": "1616664193.1584873", "lat_h": "-36.71907273180851", "lon_h": "142.20937797351252", "alt_h": "243.4553806587891"}, {"sysID_h": "2", "time_h": "1616664193.1584873", "lat_h": "-36.71538691546569", "lon_h": "142.18636041981281", "alt_h": "210.70848360023982"}, {"sysID_h": "3", "time_h": "1616664193.1584873", "lat_h": "-36.715342802396464", "lon_h": "142.2777564240451", "alt_h": "280.72262902738765"}, {"sysID_h": "4", "time_h": "1616664193.1584873", "lat_h": "-36.718430732855374", "lon_h": "142.11824349751765", "alt_h": "270.1717911308283"}, {"sysID_h": "0", "time_h": "1616664193.1584873", "lat_h": "-36.71637743399295", "lon_h": "142.14831562184483", "alt_h": "213.859304696758"}, {"sysID_h": "1", "time_h": "1616664193.1584873", "lat_h": "-36.71980524861051", "lon_h": "142.12980132216143", "alt_h": "254.33563542394694"}, {"sysID_h": "2", "time_h": "1616664193.1584873", "lat_h": "-36.71934078447395", "lon_h": "142.219660100136", "alt_h": "239.80918393657032"}, {"sysID_h": "3", "time_h": "1616664193.1584873", "lat_h": "-36.7153725217811", "lon_h": "142.16116885498164", "alt_h": "284.34822781414795"}, {"sysID_h": "4", "time_h": "1616664193.1584873", "lat_h": "-36.7189435667627", "lon_h": "142.10132189614723", "alt_h": "296.39948010459364"}]]',
                # [{"sysID": "1", "timeStamp": "1617684471.2715065", "lat": "-36.71974337404147", "lon": "142.13596552539622", "alt": "2.880566000540745", "pressure": "3.519409244329225", "signal_strength": "4.068730040878381"}, {"sysID": "2", "timeStamp": "1617684471.2715065", "lat": "-36.7181006769024", "lon": "142.24939091384286", "alt": "4.317823786002367", "pressure": "5.039998589688794", "signal_strength": "7.413431160466755"}, {"sysID": "3", "timeStamp": "1617684471.2715065", "lat": "-36.7199492605965", "lon": "142.23642475154793", "alt": "1.177569646071085", "pressure": "6.895259201840211", "signal_strength": "9.880151869917045"}, {"sysID": "4", "timeStamp": "1617684471.2715065", "lat": "-36.718801613583956", "lon": "142.1690596736946", "alt": "4.424192773165863", "pressure": "9.48927718215729", "signal_strength": "2.389026411136035"}, {"sysID_h": "1", "time_h": "1617684471.2715065", "lat_h": "-36.71739692785071", "lon_h": "142.15959244108578", "alt_h": "251.9932235819451"}, {"sysID_h": "2", "time_h": "1617684471.2715065", "lat_h": "-36.717399034806704", "lon_h": "142.18446956536053", "alt_h": "236.2285334733125"}, {"sysID_h": "3", "time_h": "1617684471.2715065", "lat_h": "-36.71830198399963", "lon_h": "142.27669752638232", "alt_h": "223.5041496360089"}, {"sysID_h": "4", "time_h": "1617684471.2715065", "lat_h": "-36.716003227062096", "lon_h": "142.22209681096115", "alt_h": "279.16353843455147"}, {"sysID_h": "1", "time_h": "1617684471.2715065", "lat_h": "-36.71566007392859", "lon_h": "142.1844256550993", "alt_h": "268.2224250269317"}, {"sysID_h": "2", "time_h": "1617684471.2715065", "lat_h": "-36.716214582039164", "lon_h": "142.2452936422035", "alt_h": "266.3069732923504"}, {"sysID_h": "3", "time_h": "1617684471.2715065", "lat_h": "-36.719758970950714", "lon_h": "142.2104154811675", "alt_h": "228.04170935409826"}, {"sysID_h": "4", "time_h": "1617684471.2715065", "lat_h": "-36.71938499459565", "lon_h": "142.26455379082907", "alt_h": "238.8156542721256"}, {"sysID_h": "1", "time_h": "1617684471.2715065", "lat_h": "-36.71926655092483", "lon_h": "142.2498537658733", "alt_h": "256.15403540386023"}, {"sysID_h": "2", "time_h": "1617684471.2715065", "lat_h": "-36.71987087359437", "lon_h": "142.15136096645068", "alt_h": "271.3279756459866"}, {"sysID_h": "3", "time_h": "1617684471.2715065", "lat_h": "-36.71840525744715", "lon_h": "142.13300384930236", "alt_h": "270.57564091552274"}, {"sysID_h": "4", "time_h": "1617684471.2715065", "lat_h": "-36.71608532175958", "lon_h": "142.1707594315208", "alt_h": "264.82388669014495"}]
                PartitionKey=str(random.randrange(10000))
        )
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

                    # update GPS_Log
                    with GPS_Log_LOCK:
                        update_GPS_Log(GPSdata)
                        distance = distance_calculation(GPS_Log)
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
                    
                    # get the GPS 
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

                    # update GPS_Log
                    with EKF_LOG_LOCK:
                        update_GPS_EKF_log(GPSdata)
                        distance = distance_calculation(GPS_Log)
                    print('Distance EKF: ',round(distance,2)," [m]")
                    print('------------------------------------------------')
                    # put data into the buffer
                    # with GlobalVals.EKF_DATA_BUFFER_MUTEX:
                    #     GlobalVals.EKF_DATA_BUFFER.append(GPSdata)

                    # set the flags for the buffer 
                    # with GlobalVals.RECIEVED_EKF_RADIO_DATA_MUTEX:
                    #     GlobalVals.RECIEVED_EKF_RADIO_DATA = True
                    
                    continue

                # if the packet is an temperature data packet 
                if recievedPacket.MessageID == 6:
                    
                    # get the temperature msg
                    tempData = CustMes.MESSAGE_TEMP()                    
                    error = tempData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: Temperature data error " + str(error) + ".\n")
                        continue
                    print("=================================================")
                    print("Temperature Data from " + str(recievedPacket.SystemID) + ":")
                    print("Temperature:" + str(round(tempData.Temperature,1)) + ", Time:" + str(tempData.Epoch) + "\n")

                    temperatureOutside = tempData.Temperature
                    
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
