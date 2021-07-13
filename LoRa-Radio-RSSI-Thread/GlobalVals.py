from threading import Lock
import numpy as np
import sys
sys.path.insert(1,'../utils/')
from common_class import *
import GlobalVariables
#=====================================================
# Global Constants 
#=====================================================
SYSID = 0
HOST = '127.0.0.1'
TIMEOUT = 10                                   # Serial port time out 
PORT = "COM17"                                  # Windows COM por
BAUDRATE = 9600                                # the baud for the serial port connection 
HANDSHAKE_BYTES = bytes([0xFF, 0x00, 0xFF])
WAITING_TIMEOUT = 2
RSSI_COMMAND = bytes(b'\xaf\xaf\x00\x00\xaf\x80\x06\x02\x00\x00\x95\x0d\x0a')
RSSI_LOG_FILE = "RSSILog.txt"

RSSI_ONLINE_CALIB = False

# Buffer
RSSI_filtered = []
distance = []
RSSI_time = []


TIMESEND = [0,1]


SOCKET_TIMEOUT = 36000
GPS_TIMEOUT = 36000
GPS_BUFFER = 2048

EndRSSISocket = False
NewRSSISocketData = False
BREAK_GPS_THREAD = False
BREAK_LORA_ALLOCATION_THREAD = False

EndRSSISocket_Mutex = Lock()
NewRSSISocketData_Mutex = Lock()
RSSIValues_Mutex = Lock()
BREAK_GPS_THREAD_MUTEX = Lock()
GPS_UPDATE_MUTEX = Lock()
BREAK_LORA_ALLOCATION_THREAD_MUTEX = Lock()

PORT_GPS = GlobalVariables.LORA_GPS_RECEIVE_SOCKET
LORA_ALLOCATION_UPDATE_MUTEX = Lock()
RSSI_ALLOCATION_SOCKET = GlobalVariables.RSSI_ALLOCATION_DISTRO_SOCKET

PORT_RSSI = GlobalVariables.LORA_RSSI_DISTRO_SOCKET
RSSI_TIMEOUT = 36000
RSSI_BUFFER = 1024

N_BALLOON = GlobalVariables.N_BALLOON
N_REAL_BALLOON = GlobalVariables.N_REAL_BALLOON
TARGET_BALLOON = 1
RSSI_CALIBRATION_SIZE = GlobalVariables.LORA_RSSI_CALIBRATION_SIZE

ALL_BALLOON = GlobalVariables.ALL_BALLOON
REAL_BALLOON = GlobalVariables.REAL_BALLOON


GPS_ALL = np.array([GPS()]*N_BALLOON)
Y = np.zeros([1,1])
X = np.zeros([1,2])
RSSI_PARAMS = GlobalVariables.RSSI_PARAMS_ALL[SYSID-1]
RSSI_CALIBRATION_FINISHED = False
SYSID = 1
LORA_ALLOCATION = 1