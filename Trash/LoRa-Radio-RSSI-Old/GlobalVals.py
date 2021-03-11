#=====================================================
# Global Constants 
#=====================================================

TIMEOUT = 1                                   # Serial port time out 
#PORT = "COM17"                                  # Windows COM por
PORT = "/dev/ttyUSB0"                          # Jetson / RasPi / Ubuntu USB COM port
BAUDRATE = 9600                                # the baud for the serial port connection 
HANDSHAKE_BYTES = bytes([0xFF, 0x00, 0xFF])
WAITING_TIMEOUT = 0.05
RSSI_COMMAND = bytes(b'\xaf\xaf\x00\x00\xaf\x80\x06\x02\x00\x00\x95\x0d\x0a')
RSSI_LOG_FILE = "RSSILog.txt"

