from threading import Lock
import numpy as np

import sys
sys.path.insert(1,'../utils/')
from common_class import *

HOST = "127.0.0.1"
BUFFER = 1024
PORT_GPS = 5280

RSSI_BUFFER = 1024
PORT_RSSI = 5120


N_BALLOON = 5
N_REAL_BALLOON = 3

ITERATION = 50
ANCHOR_LIST = np.array([2,3,4])
REAL_BALLOON_LIST = np.array([0,1,2])

LAT_REF = -36.7189
LON_REF = 142.1962
ALT_REF = 0

GPS_BUFFER = 2048
GPS_TIMEOUT = 60
RSSI_TIMEOUT = 60

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

