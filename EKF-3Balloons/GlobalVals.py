from threading import Lock
from class_def import *
import numpy as np

import sys
sys.path.insert(1,'../utils/')
from common_class import *
import GlobalVariables


HOST = '127.0.0.1'
GPS_BUFFER = 1024
IMU_BUFFER = 1024
RSSI_BUFFER = 1024
PORT_GPS = GlobalVariables.EKF_GPS_RECEIVE_SOCKET
PORT_IMU = GlobalVariables.EKF_IMU_RECEIVE_SOCKET
PORT_RSSI = GlobalVariables.EKF_RSSI_RECEIVE_SOCKET
SYSID = 1
EKF_OUTPUT_DISTRO_SOCKET = GlobalVariables.EKF_OUTPUT_DISTRO_SOCKET
EKF_LOGGER_SOCKET_TIMEOUT = 60

C_NED_ENU = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) 
C_ENU_NED = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) 

N_BALLOON = GlobalVariables.N_BALLOON
N_REAL_BALLOON = GlobalVariables.N_REAL_BALLOON
LOOPTIME = GlobalVariables.EKF_LOOPTIME

RSSI_CALIBRATION_SIZE = GlobalVariables.EKF_RSSI_CALIBRATION_SIZE
RSSI_DISTANCE_ARRAY = np.zeros([1,N_REAL_BALLOON-1])
GPS_DISTANCE_ARRAY = np.zeros([1,N_REAL_BALLOON-1])
Y = [np.zeros([1,1]), np.zeros([1,1])]
X = [np.zeros([1,2]), np.zeros([1,2])]
RSSI_PARAMS = [np.ones([1,2]), np.ones([1,2])]
RSSI_CALIBRATION_FINISHED = [False]*(N_REAL_BALLOON-1)

LAT_REF = GlobalVariables.LAT_REF
LON_REF = GlobalVariables.LON_REF
ALT_REF = GlobalVariables.ALT_REF

ANCHOR = GlobalVariables.EKF_ANCHOR
REAL_BALLOON = GlobalVariables.REAL_BALLOON

GPS_TIMEOUT = 60
IMU_TIMEOUT = 60
RSSI_TIMEOUT = 360




RSSI = np.array([RSSI()]*(N_REAL_BALLOON-1))
GPS_ALL = np.array([GPS()]*N_BALLOON)
IMU_ALL = np.array([IMU()]*N_BALLOON)
GPS_REF = GPS(None, LAT_REF, LON_REF, ALT_REF)


EKF_BUFFER = []

EKF_BUFFER_MUTEX = Lock()


BREAK_GPS_THREAD = False
BREAK_IMU_THREAD = False
BREAK_RSSI_THREAD = np.array([False]*(N_REAL_BALLOON-1))
BREAK_EKF_DISTRO_THREAD = False

BREAK_GPS_THREAD_MUTEX= Lock()
BREAK_IMU_THREAD_MUTEX = Lock()
BREAK_RSSI_THREAD_MUTEX = np.array([Lock()]*(N_REAL_BALLOON-1))
BREAK_EKF_DISTRO_THREAD_MUTEX = Lock()

RSSI_UPDATE_MUTEX = np.array([Lock()]*(N_REAL_BALLOON-1))
IMU_UPDATE_MUTEX = Lock()
GPS_UPDATE_MUTEX = Lock()
