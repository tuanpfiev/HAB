from threading import Lock
import sys
sys.path.insert(1,'../utils/')
from common_class import *
import GlobalVariables
#=====================================================
# Global Constants  
#=====================================================

# Serial Settings 
#PORT = "/dev/ttyTHS1"  # Ubuntu GPIO UART
GPS_UART_PORT = "/dev/ttyUSB7"  # Ubuntu USB UART
#GPS_UART_PORT = "COM19" 
GPS_UART_BAUDRATE = 38400
GPS_UART_TIMEOUT = 10

GPS_QUECTEL_UART_PORT = "/dev/ttyTHS1"
GPS_QUECTEL_UART_BAUDRATE = 9600
GPS_QUECTEL_UART_TIMEOUT = 10
# Networking Settings
HOST = '127.0.0.1'              # Local host address
GPS_LOGGER_SOCKET = GlobalVariables.GPS_READER_SOCKET        # Arbitrary non-privileged port for connecting to radio network
GPS_LOGGER_SOCKET_TIMEOUT = 36000   # Timeout for socket connections  

UBLOX_SIGNAL_LOSS_TIME = 2
GPS_MAX_BUFFER_LENGTH = 20
# Buffer Settings 
GPS_UART_BUFFER_SIZE = 50

# ascent rate settings
GPS_LOGGER_ASCENT_RATE_LENGTH = 10

# Log file paths
GPS_LOGGER_FILE = "GPSLog.txt"

FAKE_GPS_FLAG = False
PREDICTION_INTERVAL = 10

#=====================================================
# Global Variables  
#=====================================================

# Buffers 
GGAbuffer = []
GPSLongitude = []
GPSLatitude = []
GPSAltitude = []
GPSTimestamp = []
GPSAscentRate = 0.0
GPSAscentRateVals = []

GGA_QuectelBuffer = []


# Global flags 
EndGPSSerial = False
EndGPSSocket = False
NewGPSData = False
NewGPSSocketData = False

EndGPS_QuectelSerial = False
NEWGPS_QuectelData = False

# Mutexs 
GGABufferMutex = Lock()
GPSValuesMutex = Lock()
EndGPSSerial_Mutex = Lock()
NewGPSData_Mutex = Lock()
NewGPSSocketData_Mutex = Lock()
EndGPSSocket_Mutex = Lock()

GGA_QuectelBufferMutex = Lock()
NEWGPS_QuectelDataMutex = Lock()
EndGPS_QuectelSerialMutex = Lock()