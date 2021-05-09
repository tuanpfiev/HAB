from threading import Lock
import numpy as np
import sys
sys.path.insert(1,'../utils/')
from common_class import *
#=====================================================
# Global Constants 
#=====================================================

HOST = '127.0.0.1'
TIMEOUT = 1                                   # Serial port time out 
PORT = "COM17"                                  # Windows COM por
BAUDRATE = 9600                                # the baud for the serial port connection 
HANDSHAKE_BYTES = bytes([0xFF, 0x00, 0xFF])
WAITING_TIMEOUT = 15
RSSI_COMMAND = bytes(b'\xaf\xaf\x00\x00\xaf\x80\x06\x02\x00\x00\x95\x0d\x0a')
RSSI_LOG_FILE = "RSSILog.txt"

# Buffer
RSSI_filtered = []
distance = []
RSSI_time = []


TIMESEND = [0,1]


SOCKET_TIMEOUT = 60
GPS_TIMEOUT = 60
GPS_BUFFER = 2048

EndRSSISocket = False
NewRSSISocketData = False
BREAK_GPS_THREAD = False

EndRSSISocket_Mutex = Lock()
NewRSSISocketData_Mutex = Lock()
RSSIValues_Mutex = Lock()
BREAK_GPS_THREAD_MUTEX = Lock()
GPS_UPDATE_MUTEX = Lock()

PORT_GPS = [5200, 5210, 5290]
# PORT_RSSI = np.array([  [5100, 5110],   # EKF
#                         [5101,5111],    # LC1
#                         [5102,5112],    # LC2
#                         [5103,5113]])   # LC3
# PORT_RSSI = np.array([  [5100, 5110],   # EKF
#                         [5101,5111]])   # LC1

PORT_RSSI = np.array([  [5100, 5110],
                        [5190, 5190]])   # LC1                        
N_BALLOON = 5
N_REAL_BALLOON = 3
TARGET_BALLOON = 1
RSSI_CALIBRATION_SIZE = 10

ALL_BALLOON = np.array([1,2,3,4,5])
REAL_BALLOON = np.array([1,2,3])


GPS_ALL = np.array([GPS()]*N_BALLOON)
Y = np.zeros([1,1])
X = np.zeros([1,2])
RSSI_PARAMS = np.ones([1,2])
RSSI_CALIBRATION_FINISHED = False
SYSID = 1