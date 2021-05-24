import sys, os
sys.path.insert(1,'../utils')
from common import *
from common_class import *
import datetime
import time
import calendar

import GlobalVals

def GGA_Convert(GGAdata):
    lon = GGAdata.longitude
    lat = GGAdata.latitude
    alt = 0.0
    GPStime_hour = 0
    GPStime_min = 0
    GPStime_sec = 0
    GPSepoch = 0.0

    # if there is no data in the timestamp feild of the message then the timestamp has no type. 
    if GGAdata.data[0] != '':
        
        # set the alt
        if GGAdata.altitude != None:
            alt = GGAdata.altitude

        # get the hours, minutes, and seconds from GPS
        GPStime_hour = GGAdata.timestamp.hour
        GPStime_min = GGAdata.timestamp.minute
        GPStime_sec = GGAdata.timestamp.second

        # from the GPS calculate the seconds in the day 
        DaySecs = (GPStime_hour * 3600) + (GPStime_min * 60) + GPStime_sec
        
        # get epoch time for UTC at the start of the day 
        UTCtime = datetime.datetime.utcnow()
        UTCtimeEpoch = int(calendar.timegm(UTCtime.timetuple()))
        UTCDaySecs = (UTCtime.hour * 3600) + (UTCtime.minute * 60) + UTCtime.second
        UTCDayStart = UTCtimeEpoch - UTCDaySecs
        
        # get GPS epoch
        GPSepoch = UTCDayStart + DaySecs
        
        # if the time is conversion happens over midnight (GPS = 23:59:59, time of conversion = 00:00:00)
        # then ajust the start of the day 
        UTCtimeEpoch = UTCtimeEpoch + 60    # seems that the time gps doesn't line up exactly so a 1 minute error range helps with this 
        if GPSepoch > UTCtimeEpoch:
            UTCDayStart = UTCDayStart - 86400
            GPSepoch = UTCDayStart + DaySecs
        
        GPSepoch = float(GPSepoch)
    else:
        GPSepoch = None
    
    GPS_Data = GPS(None,lat,lon,alt,GPSepoch)

    if GPSepoch == None:
        return GPS_Data, False
    else:
        return GPS_Data, True

def updateGlobalGPS_Data(GPS_Data):

    if len(GlobalVals.GPSLongitude)> GlobalVals.GPS_MAX_BUFFER_LENGTH:
        GlobalVals.GPSLongitude.pop(0)
        GlobalVals.GPSLatitude.pop(0)
        GlobalVals.GPSAltitude.pop(0)
        GlobalVals.GPSTimestamp.pop(0)
        GlobalVals.GPSAscentRateVals.pop(0)

    GlobalVals.GPSLongitude.append(GPS_Data.lon)
    GlobalVals.GPSLatitude.append(GPS_Data.lat)
    GlobalVals.GPSAltitude.append(GPS_Data.alt)
    GlobalVals.GPSTimestamp.append(GPS_Data.epoch)
    GlobalVals.GPSAscentRateVals.append(GPS_Data.alt)
    # calculate ascent rate if enough alt vals have been recorded 
    NumAscentVals = len(GlobalVals.GPSAscentRateVals)
    if NumAscentVals >= GlobalVals.GPS_LOGGER_ASCENT_RATE_LENGTH:
        
        ascentRate = 0 
        for x in range(NumAscentVals):
            if x == 0:
                continue
            
            diff = GlobalVals.GPSAscentRateVals[x] - GlobalVals.GPSAscentRateVals[x-1]
            ascentRate = ascentRate + diff

        ascentRate = ascentRate / (NumAscentVals - 1)
        
        # store ascent rate and pop one value (running average)
        GlobalVals.GPSAscentRate = ascentRate
        GlobalVals.GPSAscentRateVals.pop(0)

def logData(gpsData,sensorID):
    logString = str(gpsData.epoch) + "," + str(gpsData.lon) + "," + str(gpsData.lat) + "," + str(gpsData.alt) + "," + str(GlobalVals.GPSAscentRate) + "," + str(sensorID) + "\n"
    timeLocal = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(gpsData.epoch))
    try:
        fileObj = open(GlobalVals.GPS_LOGGER_FILE, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error using error log file, ending error thread")
        return


    # Debug Messages 
    print("lon: " + str(round(gpsData.lon,4)) + ", lat: " + str(round(gpsData.lat,4)) + ", alt: " + str(round(gpsData.alt,2)) + ", Time: ", timeLocal)

            