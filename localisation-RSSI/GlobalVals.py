from threading import Lock
import numpy as np

import sys
sys.path.insert(1,'../utils/')
from common_class import *
import GlobalVariables

HOST = "127.0.0.1"
BUFFER = 1024
PORT_GPS = GlobalVariables.LA_R_GPS_RECEIVE_SOCKET

RSSI_BUFFER = 1024
PORT_RSSI = GlobalVariables.LA_R_RSSI_RECEIVE_SOCKET
LOOP_TIME = GlobalVariables.LA_LOOP_TIME
SIGMA_RSSI_RANGE = GlobalVariables.SIGMA_RSSI_RANGE

N_BALLOON = GlobalVariables.N_BALLOON
N_REAL_BALLOON = GlobalVariables.N_REAL_BALLOON
LEADER = GlobalVariables.LA_LEADER-1

ITERATION = GlobalVariables.LA_ITERATION
ANCHOR_LIST = GlobalVariables.LA_ANCHOR
REAL_BALLOON_LIST = GlobalVariables.REAL_BALLOON-1

LAT_REF = GlobalVariables.LAT_REF
LON_REF = GlobalVariables.LON_REF
ALT_REF = GlobalVariables.ALT_REF

GPS_BUFFER = 2048
GPS_TIMEOUT = 60
RSSI_TIMEOUT = 360

RSSI_MATRIX = np.array([np.array([RSSI()]*N_REAL_BALLOON)]*N_REAL_BALLOON)
GPS_ALL =  np.array([GPS()]*N_BALLOON)
GPS_REF = GPS(None,LAT_REF, LON_REF, ALT_REF)  # GPS of GMAC
POS_XYZ = np.array([POS_XYZ()]*N_BALLOON)

BREAK_RSSI_THREAD = False
BREAK_GPS_THREAD = False

BREAK_RSSI_THREAD_MUTEX = Lock()
BREAK_GPS_THREAD_MUTEX = Lock()

POSXYZ_UPDATE_MUTEX = Lock()
RSSI_UPDATE_MUTEX = Lock()
GPS_UPDATE_MUTEX = Lock()

