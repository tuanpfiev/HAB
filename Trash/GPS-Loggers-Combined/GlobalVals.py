from threading import Lock

#=====================================================
# Global Constants  
#=====================================================

# Serial Settings 
#PORT = "/dev/ttyTHS1"  # Ubuntu GPIO UART
GPS_UART_PORT = "/dev/ttyUSB7"  # Ubuntu USB UART
#GPS_UART_PORT = "COM19" 
GPS_UART_BAUDRATE = 38400
GPS_UART_TIMEOUT = 1

# Networking Settings
HOST = '127.0.0.1'              # Local host address
GPS_UBLOX_LOGGER_SOCKET = 5090        # Arbitrary non-privileged port for connecting to radio network
GPS_UBLOX_LOGGER_SOCKET_TIMEOUT = 60   # Timeout for socket connections  

GPS_QUECTEL_LOGGER_SOCKET = 5091
GPS_QUECTEL_LOGGER_SOCKET_TIMEOUT = 60

GPS_LOGGER_SOCKET = 5001
GPS_LOGGER_SOCKET_TIMEOUT = 60

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

GPS_UBLOX_DATA_BUFFER = []
GPS_QUECTEL_DATA_BUFFER= [] 

# Global flags 
EndGPSSerial = False
EndGPSSocket = False
NewGPSData = False
NewGPSSocketData = False

BREAK_GPS_LOGGER_UBLOX_THREAD = False
BREAK_GPS_LOGGER_QUECTEL_THREAD = False
# Mutexs 
GGABufferMutex = Lock()
GPSValuesMutex = Lock()
EndGPSSerial_Mutex = Lock()
NewGPSData_Mutex = Lock()
NewGPSSocketData_Mutex = Lock()
EndGPSSocket_Mutex = Lock()

GPS_UBLOX_DATA_BUFFER_MUTEX = Lock()
GPS_QUECTEL_DATA_BUFFER_MUTEX = Lock()

BREAK_GPS_LOGGER_UBLOX_THREAD_MUTEX = Lock()
BREAK_GPS_LOGGER_QUECTEL_THREAD_MUTEX = Lock()