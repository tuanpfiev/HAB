from threading import Lock
#=====================================================
# Global Constants 
#=====================================================

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


SOCKET_TIMEOUT = 10

EndRSSISocket = False
NewRSSISocketData = False

EndRSSISocket_Mutex = Lock()
NewRSSISocketData_Mutex = Lock()
RSSIValues_Mutex = Lock()
