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
import RSSI_Handler
import TemperatureHandler
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
            distanceMatrix[i][j] = round(distance_calculation(gpsAll[GlobalVals.N_REAL_BALLOON+i],gpsAll[GlobalVals.N_REAL_BALLOON+j]),nRound)
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
nRealBalloon = GlobalVals.N_REAL_BALLOON
balloonPaths = []
baloonPathAll = [Path("Balloon_254.csv", 1), Path("Balloon_253.csv", 2), Path("Balloon_255.csv", 3)]
baloonPathAllOriginal = baloonPathAll[:]
nTruncate = 20
# Can be fires or whatever was targeted by the predictions for each balloon
# targets = [-36.373326870216395, 142.36570090762967, -36.473326870216395, 142.36570090762967, -36.573326870216395, 142.36570090762967]
targets = [-37.67263447464716, 145.01434427800544,-37.678249800742954, 145.0194354883337,-37.66112634441829, 145.04196260796772]
targetLocation = [GPS(None, targets[0], targets[1], 0),
                  GPS(None, targets[2], targets[3], 0),
                  GPS(None, targets[4], targets[5], 0)]
nFire = 2
fireLocation = [-37.68419307606098, 145.03398691655838, -37.65732515799401, 145.0160016938692]
predDuration = []
predMaxAlt = np.zeros(nRealBalloon)

for i in range(len(baloonPathAll)):
    baloonPathAll[i].trajectory = dataTruncate(baloonPathAll[i].trajectory,nTruncate,1)

for i in range(len(baloonPathAll)):
    predDuration.append(baloonPathAll[i].trajectory[-1].epoch - baloonPathAll[i].trajectory[0].epoch)
    for j in range(len(baloonPathAll[i].trajectory)):
        each_balloon = {
                    'idP': str(baloonPathAll[i].trajectory[j].sysID),
                    'latP': str(baloonPathAll[i].trajectory[j].lat),
                    'lonP': str(baloonPathAll[i].trajectory[j].lon)
                }
        balloonPaths.append(each_balloon)
        predMaxAlt[i] = max(predMaxAlt[i], baloonPathAll[i].trajectory[j].alt)

# for i in range(len(baloonPathAll)):
#     baloonPathAll[i].trajectory = dataTruncate(baloonPathAll[i].trajectory,nTruncate,1)

# for i in range(len(baloonPathAll)):
#     for j in range(len(baloonPathAll[0].trajectory)):
#         each_balloon = {
#                     'idP': str(baloonPathAll[i].trajectory[j].sysID),
#                     'tP': str(baloonPathAll[i].trajectory[j].epoch),
#                     'latP': str(baloonPathAll[i].trajectory[j].lat),
#                     'lonP': str(baloonPathAll[i].trajectory[j].lon),
#                     'altP': str(baloonPathAll[i].trajectory[j].alt)
#                 }
#         balloonPaths.append(each_balloon)



global  count_history,pathHistory,count_t,sumPositionError,sumPositionErrorSquare,sumTargetOffset, nSample
count_history = 1
pathHistory = []
count_t = 0
sumPositionError = np.zeros([nRealBalloon,1])
sumPositionErrorSquare = np.zeros([nRealBalloon,1])
sumTargetOffset = np.zeros([nRealBalloon,1])
nSample = 0

def update_temperature(tempur):
    index = tempur.sysID
    GlobalVals.TEMPERATURE_ALL[index-1]=tempur

def update_GPS_Log(gps_data):
    index = gps_data.SystemID
    try:    
        GlobalVals.GPS_ALL[index-1]= GPS(gps_data.SystemID, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here-update_GPS_Log')

def update_EKF_Log(EKF_Data):
    index = EKF_Data.SystemID
    try:    
        GlobalVals.GPS_ALL[index-1+GlobalVals.N_REAL_BALLOON]= GPS(EKF_Data.SystemID+GlobalVals.N_REAL_BALLOON, EKF_Data.Latitude, EKF_Data.Longitude, EKF_Data.Altitude,EKF_Data.Epoch)
    except:
        print('here-update_EKF_Log')


def distance_calculation(gps_data):
    p1 = lla2ecef(gps_data[0].lat,gps_data[0].lon,gps_data[0].alt)
    p2 = lla2ecef(gps_data[1].lat,gps_data[1].lon,gps_data[1].alt)
    return np.linalg.norm(p1-p2)

def distance_calculation(gps1,gps2):
    p1 = lla2ecef(gps1.lat,gps1.lon,gps1.alt)
    p2 = lla2ecef(gps2.lat,gps2.lon,gps2.alt)
    return np.linalg.norm(p1-p2)

def gps_lambda_handler(credentials):
    global count_t, count_history, pathHistory, nSample

    stream_name = 'RMITballoon_Data'
    k_client = boto3.client('kinesis', 
                            region_name='ap-southeast-2',
                            aws_access_key_id=credentials['AccessKeyId'],
                            aws_secret_access_key=credentials['SecretKey'],
                            aws_session_token=credentials['SessionToken'])
    nRealBalloon = GlobalVals.N_REAL_BALLOON

    time.sleep(5)
    while True:

        with GlobalVals.GPS_LOG_MUTEX:
            GPS_Log = copy.deepcopy(GlobalVals.GPS_ALL)

        # with GlobalVals.TEMPERATURE_UPDATE_MUTEX:
        #     tempAll = copy.deepcopy(GlobalVals.TEMPERATURE_ALL)

        with GlobalVals.PACKET_STATS_AWS_MUTEX:
            packetStatsLogPercent = copy.deepcopy(GlobalVals.PACKET_STATS_AWS)
        nSample = nSample + 1
        if count_t == 0:
            aws_message = balloonPaths[:]
            initial_message = {
                'nBalloon': str(nRealBalloon),
                'targets': str(targets),
                'nFire': str(nFire),
                'fireLocation': str(fireLocation),
                'flightDuration': str(predDuration),
                'maxAlt': str(predMaxAlt.tolist())
            }
            aws_message.append(initial_message)
        else:
            # Create predicted paths from initial predictions (needs to update)
            predictedPaths = []
            for i in range(len(baloonPathAll)):
                each_balloon = {
                    'idP': str(baloonPathAll[i].trajectory[0].sysID),
                    'latP': str(GPS_Log[i].lat ),
                    'lonP': str(GPS_Log[i].lon)
                }
                predictedPaths.append(each_balloon)
                for j in range(count_t*10, len(baloonPathAll[i].trajectory)):
                    each_balloon = {
                        'idP': str(baloonPathAll[i].trajectory[j].sysID),
                        'latP': str(baloonPathAll[i].trajectory[j].lat),
                        'lonP': str(baloonPathAll[i].trajectory[j].lon)
                    }
                    predictedPaths.append(each_balloon)
            aws_message = predictedPaths[:]

            t0 = time.time()

            for i in range(len(GPS_Log)):
                if i < nRealBalloon:
                    positionENU_RelativeTarget = positionENU(GPS_Log[i],targetLocation[i])
                    targetOffset = math.sqrt(positionENU_RelativeTarget[0]**2 + positionENU_RelativeTarget[1]**2)
                    meanEKF = 0 
                    covarEKF = 0
                else:
                    positionENU_RelativeEKF = positionENU(GPS_Log[i],GPS_Log[i-nRealBalloon])
                    targetOffset = math.sqrt(positionENU_RelativeEKF[0]**2+positionENU_RelativeEKF[1]**2)
                    
                    sumPositionError[i-nRealBalloon][0] = sumPositionError[i-nRealBalloon][0] + targetOffset
                    sumPositionErrorSquare[i-nRealBalloon][0] = sumPositionErrorSquare[i-nRealBalloon][0] + targetOffset **2
                    meanEKF = sumPositionError[i-nRealBalloon][0]/nSample
                    covarEKF = math.sqrt(sumPositionErrorSquare[i-nRealBalloon][0]/nSample)
                    
                if i<nRealBalloon:
                    comms = 0.999-packetStatsLogPercent[i]
                else:
                    comms = 0.999-packetStatsLogPercent[i-nRealBalloon]

                each_balloon = {
                    'id': str(GPS_Log[i].sysID),
                    't': str(t0),
                    'lat': str(GPS_Log[i].lat ),
                    'lon': str(GPS_Log[i].lon),
                    'alt': str(GPS_Log[i].alt),
                    'mEKF': str(meanEKF),
                    'cEKF': str(covarEKF),
                    'tar': str(targetOffset),
                    'comms': str(comms)
                    # 'comms': str(1)

                }
                aws_message.append(each_balloon)


        # response = k_client.put_record(
        #         StreamName=stream_name,
        #         Data=json.dumps(aws_message),
        #         PartitionKey='telemetryData'
        # )
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        print(json.dumps(aws_message))
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        # print(aws_message)
        print("Publishing to AWS Kinesis Data ...")
        print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")

        count_t = count_t + 1
        time.sleep(3)

def cognito_login(username, password):
    region = 'ap-southeast-2'
    clientID = '6s77pp2bq57348s96kujudbes5'
    userPoolID = 'ZHCg4nUow'
    identityPoolID = 'b8f994a8-9f48-4efb-ac58-e4101703ee87'
    login = 'cognito-idp.' + region + '.amazonaws.com/' + region + "_" + userPoolID
    cogIdp_client = boto3.client('cognito-idp', region_name=region)
    cog_client = boto3.client('cognito-identity', region_name=region)
    
    # Initiate the authentication request
    print('Logging In')
    authResult = cogIdp_client.initiate_auth(
        AuthFlow = 'USER_PASSWORD_AUTH',
        AuthParameters = {
            'USERNAME': username,
            'PASSWORD': password,
            },
        ClientId = clientID
        )
    # Get a cognito identity
    print('Getting ID')
    idResult = cog_client.get_id(
        IdentityPoolId = region + ':' + identityPoolID,
        Logins = {
            login: authResult['AuthenticationResult']['IdToken'],
            }
        )
    # Get credentials for the identity
    print('Getting Credentials')
    credResult = cog_client.get_credentials_for_identity(
        IdentityId = idResult['IdentityId'],
        Logins = {
            login: authResult['AuthenticationResult']['IdToken']
            }
        )
    gps_lambda_handler(credResult['Credentials'])       
#=====================================================
# Main function  
#=====================================================
global countT, countE
countT = 0
countE = 0
def main():
    global countT, countE

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
                        print ("Radio Network Main: GPS data error " + str(error) )
                        continue
                    
                    

                    # print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                    # print("Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,2)) + ", Time:" + str(GPSdata.GPSTime) + "\n")

                    # set the system id for the GPS data
                    # GPSdata.SystemID = recievedPacket.SystemID
                    countT = countT + 1
                    GPSdata.SystemID = int(GPSdata.SystemID)
                    if GPSdata.SystemID != recievedPacket.SystemID:
                        print("GPS SysID mismatched")
                        countE = countE + 1
                        print(countE,"/",countT,round(countE/countT,2))
                        continue

                    if not GPSHandler.GPS_FormatCheck(GPSdata):
                        print("GPS message via RFD900 was broken. Discard it...") 
                        continue

                    # print("GPS Data from " + str(recievedPacket.SystemID) + ":")
                    # print("Lon:" + str(round(GPSdata.Longitude,3)) + ", Lat:" + str(round(GPSdata.Latitude,3)) + ", Alt:" + str(round(GPSdata.Altitude,2)) + ", Time:" + str(GPSdata.GPSTime) + "\n")

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
                        print ("Radio Network Main: EKF data error " + str(error) )
                        continue

                    # set the system id for the GPS data
                    EKF_Data.SystemID = recievedPacket.SystemID
                    
                    if not EKFHandler.EKF_FormatCheck(EKF_Data):
                        print("EKF message via RFD900 was broken. Discard it...") 
                        continue

                    # update GPS_Log
                    with GlobalVals.EKF_LOG_MUTEX:
                        update_EKF_Log(EKF_Data)
                        distance = distanceEKF_MatrixCalculation(GlobalVals.GPS_ALL,0,)

                    print("==================================================================================================")
                    print("EKF EKF EKF " + str(recievedPacket.SystemID) +str(recievedPacket.SystemID) +str(recievedPacket.SystemID) + ":" + " Lon:" + str(round(EKF_Data.Longitude,3)) + ", Lat:" + str(round(EKF_Data.Latitude,3)) + ", Alt:" + str(round(EKF_Data.Altitude,1)) + ", Time:" + str(round(EKF_Data.Epoch,1)))
                    print('Distance from EKF [m]:\n',distance)

                    continue

                # if the packet is an temperature data packet 
                # Temperature
                if recievedPacket.MessageID == 6:

                    # get the RSSI data
                    temperatureData = CustMes.MESSAGE_TEMP()                    
                    error = temperatureData.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: temperature data error " + str(error) )
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


                 # RSSI
                if recievedPacket.MessageID == 7:

                    # get the RSSI data
                    RSSI_Data = CustMes.MESSAGE_RSSI()                    
                    error = RSSI_Data.bytes_to_data(recievedPacket.Payload)
                    if error != 0:
                        print ("Radio Network Main: RSSI data error " + str(error) )
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
                    
                    print("RSSI Data from " + str(recievedPacket.SystemID) + ": " + "RSSI Distance:" + str(round(RSSI_Data.Distance,1)) + ", Filtered RSSI: " + str(round(RSSI_Data.FilteredRSSI,1)) + ", TargetPayloadID: " + str(RSSI_Data.TargetPayloadID) + ", Time: " + str(RSSI_Data.Epoch) + ", SysID: " + str(RSSI_Data.SystemID))

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
                        print ("Radio Network Main: RSSI Allocation data error " + str(error) )
                        continue
                    
                    # set the system id for the GPS data
                    RSSI_AllocationData.SystemID = recievedPacket.SystemID
                    
                    if not RSSI_Handler.RSSI_AllocationFormatCheck(RSSI_AllocationData):
                        print("RSSI Allocation message via RFD900 was broken. Discard it...")
                        continue
                    
                    print("RSSI Allocation Data from " + str(recievedPacket.SystemID) + ":" + "Pair:" + str(int(RSSI_AllocationData.Pair)))

                    # put data into the buffer
                    with GlobalVals.RSSI_DATA_ALLOCATION_BUFFER_MUTEX:
                        # if len(GlobalVals.RSSI_DATA_ALLOCATION_BUFFER)>5:
                        #     GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.pop(0)
                        GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.append(int(RSSI_AllocationData.Pair))

                    # set the flags for the buffer 
                    with GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA_MUTEX:
                        GlobalVals.RECIEVED_RSSI_ALLOCATION_RADIO_DATA = True
                
                    continue


        with GlobalVals.PACKET_STATS_LOG_MUTEX:
            packetStatsLogTmp = copy.deepcopy(GlobalVals.PACKET_STATS_LOG)

        with GlobalVals.PACKET_STATS_AWS_MUTEX:
            GlobalVals.PACKET_STATS_AWS = packetStatsLogTmp
            
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
    # AWS_GPS_Thread = Thread(target = gps_lambda_handler, args=())
    AWS_GPS_Thread = Thread(target = cognito_login, args=(GlobalVals.UNAME,GlobalVals.PWD))
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
