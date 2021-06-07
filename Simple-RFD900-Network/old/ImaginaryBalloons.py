import CustMes
import math
import time
import GlobalVals
import csv
from threading import Lock


class tMercator:

    def __init__(self, Latitude: float, Longitude: float, Altitude: float, Epoch: int):

        self.Latitude = Latitude
        self.Longitude = Longitude
        self.Altitude = Altitude
        self.Epoch = Epoch


class ImaginaryBalloon:

    def __init__(self, path, SystemID):

        self.path = path
        self.SystemID = SystemID
        self.trajectory = []
        
        with open(self.path) as csv_file:
            self.reader = csv.reader(csv_file, delimiter=',')
            
            for row in self.reader:

                mercator = tMercator(float(row[1]), float(row[2]), float(row[3]), int(row[0]))
                self.trajectory.append(mercator)

        self.epochs = [self.trajectory[0].Epoch, self.trajectory[-1].Epoch]

    def get_tMercator(self, query_epoch: int):

        if query_epoch < self.epochs[0]:

            return self.trajectory[0]

        if query_epoch > self.epochs[-1]:

            return self.trajectory[-1]

        idx = query_epoch - self.epochs[0]

        return self.trajectory[idx]


def ImaginaryBalloons():   

    # Balloons = [ImaginaryBalloon("Balloon_255.csv", 255), ImaginaryBalloon("Balloon_254.csv", 254), ImaginaryBalloon("Balloon_253.csv", 253)]
    Balloons = [ImaginaryBalloon("Balloon_254.csv", 254), ImaginaryBalloon("Balloon_253.csv", 253)]

    break_condition = False

    while True:

        for Balloon in Balloons:

            interpolated_tMercator = Balloon.get_tMercator(int(time.time()))

            GPSdata = CustMes.MESSAGE_GPS()

            GPSdata.Longitude = interpolated_tMercator.Longitude
            GPSdata.Latitude = interpolated_tMercator.Latitude
            GPSdata.GPSTime = interpolated_tMercator.Epoch
            GPSdata.SystemID = Balloon.SystemID
            GPSdata.Altitude = interpolated_tMercator.Altitude

            #print(str(GPSdata.Latitude) + ', ' + str(GPSdata.Longitude) + ', ' + str(GPSdata.GPSTime) + ', ' + str(GPSdata.SystemID) + ', ' + str(GPSdata.Altitude))

            with GlobalVals.GPS_DATA_BUFFER_MUTEX:
                GlobalVals.GPS_DATA_BUFFER.append(GPSdata)
            
            with GlobalVals.RECIEVED_GPS_LOCAL_DATA_MUTEX:
                GlobalVals.RECIEVED_GPS_LOCAL_DATA = True

            with GlobalVals.BREAK_IMAGINARY_BALLOONS_MUTEX:

                if GlobalVals.BREAK_IMAGINARY_BALLOONS_THREAD:

                    break_condition = True
                    break

        if break_condition:

            break

        time.sleep(1)