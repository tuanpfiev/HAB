import time
import datetime
from dataclasses import dataclass
from threading import Lock

import numpy as np
import sys
sys.path.insert(1,'../utils/')
from common_class import *
import GlobalVariables

#=====================================================
# Global Constants 
#=====================================================
N_BALLOON = GlobalVariables.N_BALLOON
N_REAL_BALLOON = GlobalVariables.N_REAL_BALLOON
REAL_BALLOON = GlobalVariables.REAL_BALLOON
# serial settings 
TIMEOUT = 0.5                       # Serial port time out 
PORT = "COM8"                       # Windows COM por
#PORT = "/dev/ttyUSB0"              # Jetson / RasPi / Ubuntu USB COM port
BAUDRATE = 57600                    # the baud for the serial port connection 

# Socket settings 
HOST = '127.0.0.1'
GPS_LOGGER_SOCKET = GlobalVariables.GPS_READER_SOCKET
GPS_DISTRO_SOCKET =GlobalVariables.GPS_ALL_DISTRO
N_NODE_PUBLISH = len(GPS_DISTRO_SOCKET)

GPS_LOGGER_SOCKET_TIMEOUT = 60


IMU_LOGGER_SOCKET = GlobalVariables.IMU_READER_SOCKET
IMU_DISTRO_SOCKET = GlobalVariables.IMU_DISTRO_SOCKET
IMU_LOGGER_SOCKET_TIMEOUT = 60

EKF_GPS_LOGGER_SOCKET = GlobalVariables.EKF_GPS_LOGGER_SOCKET
EKF_GPS_LOGGER_SOCKET_TIMEOUT = 60

TEMP_LOGGER_SOCKET = GlobalVariables.TEMP_LOGGER_SOCKET
TEMP_LOGGER_SOCKET_TIMEOUT = 60

RSSI_LOGGER_SOCKET = GlobalVariables.RSSI_LOGGER_SOCKET
RSSI_LOGGER_SOCKET_TIMEOUT = 60

RSSI_DISTRO_SOCKET = GlobalVariables.RSSI_DISTRO_SOCKET
RSSI_ALLOCATION_DISTRO_SOCKET = GlobalVariables.RSSI_ALLOCATION_DISTRO_SOCKET
N_RSSI_NODE_PUBLISH = len(RSSI_DISTRO_SOCKET)
NEXT_PAIR = 1

EXPERIMENT_TIME = GlobalVariables.EXPERIMENT_TIME
LORA_PAIR_NUM = GlobalVariables.LORA_PAIR_NUM

# other settings 
PACKET_BUFFER_IN_MAX_SIZE = 200      # This is the maximum number of packets that can be stored in the buffers 
PACKET_CHECK_INTERVAL = 0.001        # the time in seconds the thread is paused waiting for more packets to arrive  
SYSTEM_ID = 2                       # ID of this system on the network (max 255) 
ERROR_LOG_FILE = "../datalog/ErrorLog.txt"
PING_LOG_FILE = "../datalog/PingLog.txt"
PACKET_STATS_FILE = "../datalog/PacketStats.txt"
GROUND_STATION_LOG_FILE = "../datalog/GSLog.txt"
PING_WAIT_TIME = 0.1
PING_LOOP_LIMIT = 50
PING_INTERVAL = 10
PACKET_STATS_INTERVAL = 60

#=====================================================
# Global Variables  
#=====================================================

# Global Lists / Buffers
PACKET_BUFFER_IN = []               # The buffer for in coming packets 
PACKET_BUFFER_OUT = []              # The buffer for out going packets 
PACKET_PING_BUFFER = []
PACKET_ERROR_QUE = []               # The buffer for error reporting 
SEQ_TRACKERS = []                   # The list containing all sequence trackers 
GPS_DATA_BUFFER = []

IMU_DATA_BUFFER = []
AWS_GPS_DATA_BUFFER = [] 
EKF_GPS_DATA_BUFFER = []
EKF_GPS_ALL = np.array([GPS()])
RSSI_ALL = np.array([RSSI()])
TEMPERATURE_ALL = np.array([TEMPERATURE()])

TEMP_DATA_BUFFER = []
RSSI_DATA_BUFFER = []

RSSI_ALLOCATION = np.array([np.array([False]*N_REAL_BALLOON)]*N_REAL_BALLOON)
RSSI_DATA_ALLOCATION_BUFFER = []

GPS_ARRAY_RADIO_CHECK = [[GPS()]*N_REAL_BALLOON]



# Global Lists / Buffers Mutexes 
PACKET_BUFFER_IN_MUTEX = Lock()
PACKET_BUFFER_OUT_MUTEX = Lock()
PACKET_PING_BUFFER_MUTEX = Lock()
ERROR_DETECTED_MUTEX = Lock()
SEQ_TRACKERS_MUTEX = Lock()
GPS_DATA_BUFFER_MUTEX = Lock()

IMU_DATA_BUFFER_MUTEX = Lock()
AWS_GPS_DATA_BUFFER_MUTEX = Lock()

TEMP_DATA_BUFFER_MUTEX = Lock()
RSSI_DATA_BUFFER_MUTEX = Lock()
RECIEVED_RSSI_LOCAL_DATA_MUTEX = Lock()
RSSI_DATA_ALLOCATION_BUFFER_MUTEX = Lock()
RECIEVED_RSSI_ALLOCATION_RADIO_DATA_MUTEX = Lock()

# Global Flags 
SEND_PACKETS = False                # This flag informs the network manager to send all packets in the out going buffer
RECIEVED_PACKETS = False            # This flag indicates that the network manager has recieved packets which are now stored in the buffer 
RECIEVED_PING = True
BREAK_NETWORK_THREAD = False        # This flag is used to make the network manager thread gracefully close
ERROR_DETECTED = False              # This flag indicates that an error has been found in the packets, the error should be in the packet error que
BREAK_ERROR_THREAD = False
BREAK_GPS_LOGGER_THREAD = False
BREAK_GPS_DISTRO_THREAD = False
BREAK_PING_THREAD = False
RECIEVED_GPS_LOCAL_DATA = False
RECIEVED_GPS_RADIO_DATA = False
BREAK_IMAGINARY_BALLOONS_THREAD = False

BREAK_IMU_LOGGER_THREAD = False
RECIEVED_IMU_LOCAL_DATA = False
BREAK_IMU_DISTRO_THREAD = False
RECIEVED_IMU_RADIO_DATA = False
RECEIVED_RSSI_RADIO_DATA = [False]*(N_REAL_BALLOON-1)

BREAK_EKF_GPS_LOGGER_THREAD = False
BREAK_AWS_GPS_THREAD = False
RECIEVED_EKF_GPS_LOCAL_DATA = False
BREAK_RSSI_LOGGER_THREAD = [False]*(N_REAL_BALLOON-1)

BREAK_TEMP_LOGGER_THREAD = False
RECIEVED_RSSI_RADIO_DATA = False
RECIEVED_RSSI_LOCAL_DATA = False
BREAK_RSSI_DISTRO_THREAD = False
BREAK_RSSI_ALLOCATION_DISTRO_THREAD = False
RECIEVED_RSSI_ALLOCATION_RADIO_DATA = False
# Global Flags Mutexes  
SEND_PACKETS_MUTEX = Lock()
RECIEVED_PACKETS_MUTEX = Lock()
RECIEVED_PING_MUTEX = Lock()
BREAK_NETWORK_THREAD_MUTEX = Lock()
PACKET_ERROR_MUTEX = Lock()
BREAK_ERROR_THREAD_MUTEX = Lock()
BREAK_GPS_LOGGER_THREAD_MUTEX = Lock()
BREAK_GPS_DISTRO_THREAD_MUTEX = Lock()
BREAK_PING_THREAD_MUTEX = Lock()
RECIEVED_GPS_LOCAL_DATA_MUTEX = Lock()
RECIEVED_GPS_RADIO_DATA_MUTEX = Lock()
PACKET_COUNT_MUTEX = Lock()
BREAK_IMAGINARY_BALLOONS_MUTEX = Lock()

BREAK_IMU_LOGGER_THREAD_MUTEX = Lock()
RECIEVED_IMU_LOCAL_DATA_MUTEX = Lock()
BREAK_IMU_DISTRO_THREAD_MUTEX = Lock()
RECIEVED_IMU_RADIO_DATA_MUTEX = Lock()
BREAK_EKF_GPS_LOGGER_THREAD_MUTEX = Lock()
BREAK_RSSI_LOGGER_THREAD_MUTEX = [Lock()]*(N_REAL_BALLOON-1)
RSSI_UPDATE_MUTEX = [Lock()]*(N_REAL_BALLOON-1)
BREAK_RSSI_DISTRO_THREAD_MUTEX = Lock()

EKF_GPS_DATA_BUFFER_MUTEX = Lock()
RECIEVED_EKF_GPS_LOCAL_DATA_MUTEX = Lock()

BREAK_TEMP_LOGGER_THREAD_MUTEX = Lock()
RECIEVED_RSSI_RADIO_DATA_MUTEX = Lock()

RSSI_ALLOCATION_MUTEX = Lock()
BREAK_RSSI_ALLOCATION_DISTRO_THREAD_MUTEX = Lock()
# Global Values 
MESSAGE_ID = 0                      # The current message id used for sending messages
PACKET_COUNT = 0




#=====================================================
# Global data classes
#=====================================================

# dataclass for reporting errors or missing packets  
# ErrorType:
# 1 = Packet frame check failed 
# 2 = Packet Payload check failed 
# 3 = Broken packet sequence
@dataclass
class ErrorRecord:
    ErrorType: int = 0
    ErrorCode: int = 0
    OriginID: int = 0
    Timestamp: float = 0

# dataclass for tracking sequence numbers (packet loss)
@dataclass
class sequenceTracker:
    CurrentNumber: int = 0
    SystemID: int = 0
    
    def NextNumber(self, seqNumber):

        # check if the sequence number is in range 
        if seqNumber > 255 or seqNumber < 0:
            return -1

        # calculate the expected sequence number 
        seqNext = self.CurrentNumber + 1
        if self.CurrentNumber == 255:
            seqNext = 0

        # if the expected sequence number matches store it and return 0
        if seqNext == seqNumber:
            self.CurrentNumber = seqNext
            return 0
        else:
            # if it doesn't match find the number of missing packets
            # and store seqNumber as the current number 
            seqDiff = seqNumber - self.CurrentNumber
            if seqDiff <= 0:
                x = seqDiff
                seqDiff = 256 + x

            self.CurrentNumber = seqNumber
            return seqDiff

#=====================================================
# Global functions 
#=====================================================

# function for reporting packet errors 
def reportError(ErrorType, ErrorCode, OriginID):
    
    global ERROR_DETECTED
    global PACKET_ERROR_QUE

    # make an error report 
    errorReport = ErrorRecord()
    errorReport.ErrorType = ErrorType
    errorReport.ErrorCode = ErrorCode
    errorReport.OriginID = OriginID
    errorReport.Timestamp = time.time()

    # add report to que 
    with PACKET_ERROR_MUTEX:
        PACKET_ERROR_QUE.append(errorReport)
    
    with ERROR_DETECTED_MUTEX:
        ERROR_DETECTED = True
