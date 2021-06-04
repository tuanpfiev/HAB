import time
import serial
from threading import Lock

# import files 
import GlobalVals
import CustMes    
    
GlobalVals.PORT = "/dev/ttyUSB0"
try:
    serial_port = serial.Serial(
        port=GlobalVals.PORT,
        baudrate=GlobalVals.BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=GlobalVals.TIMEOUT,
    )
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
except Exception as e:
    print("Network Manager: Unable to initiate serial port. Now breaking thread.")
    print("Network Manager: Exception: " + str(e.__class__))
    connected = False