from threading import Lock
from class_def import *
import numpy as np

import sys
sys.path.insert(1,'../utils/')
from common_class import *


HOST = '127.0.0.1'
GPS_BUFFER = 1024
IMU_BUFFER = 1024
RSSI_BUFFER = 1024
PORT_GPS = 5002
PORT_IMU = 5003
PORT_RSSI = [5100, 5110]
SYSID = 1
EKF_GPS_DISTRO_SOCKET = 5051
EKF_GPS_LOGGER_SOCKET_TIMEOUT = 60

C_NED_ENU = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) 
C_ENU_NED = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) 

N_BALLOON = 5
N_REAL_BALLOON = 3
LOOPTIME = 0.02
RSSI_CALIBRATION_SIZE = 10
RSSI_DISTANCE_ARRAY = np.zeros([1,N_REAL_BALLOON-1])
GPS_DISTANCE_ARRAY = np.zeros([1,N_REAL_BALLOON-1])
Y = [np.zeros([1,1]), np.zeros([1,1])]
X = [np.zeros([1,2]), np.zeros([1,2])]
RSSI_PARAMS = [np.ones([1,2]), np.ones([1,2])]
RSSI_CALIBRATION_FINISHED = [False]*(N_REAL_BALLOON-1)

LAT_REF = -36.7189
LON_REF = 142.1962
ALT_REF = 0

ANCHOR = np.array([1,2,3,4,5])
REAL_BALLOON = np.array([1,2,3])

GPS_TIMEOUT = 60
IMU_TIMEOUT = 60
RSSI_TIMEOUT = 60




RSSI = np.array([RSSI()]*(N_REAL_BALLOON-1))
GPS_ALL = np.array([GPS()]*N_BALLOON)
IMU_ALL = np.array([IMU()]*N_BALLOON)
GPS_REF = GPS(None, LAT_REF, LON_REF, ALT_REF)


LLA_EKF_BUFFER = []

LLA_EKF_BUFFER_MUTEX = Lock()


BREAK_GPS_THREAD = False
BREAK_IMU_THREAD = False
BREAK_RSSI_THREAD = np.array([False]*(N_REAL_BALLOON-1))
BREAK_EKF_GPS_DISTRO_THREAD = False

BREAK_GPS_THREAD_MUTEX= Lock()
BREAK_IMU_THREAD_MUTEX = Lock()
BREAK_RSSI_THREAD_MUTEX = np.array([Lock()]*(N_REAL_BALLOON-1))
BREAK_EKF_GPS_DISTRO_THREAD_MUTEX = Lock()

RSSI_UPDATE_MUTEX = np.array([Lock()]*(N_REAL_BALLOON-1))
IMU_UPDATE_MUTEX = Lock()
GPS_UPDATE_MUTEX = Lock()
