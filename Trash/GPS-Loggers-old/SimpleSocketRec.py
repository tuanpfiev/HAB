import socket
import GlobalVals 
import struct
      
socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
socket_logger.connect((GlobalVals.HOST, GlobalVals.GPS_LOGGER_SOCKET))
socket_logger.settimeout(2)

synced = False
syncA = False
syncB = False

bufferRead = 1
breakLoop = False

while not breakLoop:
    
    # reset buffer read when not synced 
    if not synced:
        bufferRead = 1

    # read the socket 
    try:
        data_bytes = socket_logger.recv(bufferRead)
    except:
        print("Connection error.")
        breakLoop = True
        continue
    
    # if there is nothing in the socket then it has timed out 
    if len(data_bytes) == 0:
        continue
    
    # for the sync bytes (0xAA 0x55)
    if not synced:
        if data_bytes[0] == 0xAA and not syncA:
            syncA = True
            bufferRead = 1
            continue
        elif data_bytes[0] == 0x55 and syncA:
            syncB = True
            bufferRead = 32
        else:
            syncA = False
            syncB = False
            bufferRead = 1
            continue 
        
        # if both bytes have been found in order then the socket buffer is synced 
        if syncA and syncB:
            synced = True
            syncA = False
            syncB = False
            continue 
    
    # once it is scyned read the rest of the data 
    if synced and bufferRead == 32:

        # convert payload values back to double
        LongitudeTuple = struct.unpack('!d',data_bytes[0:8])
        LatitudeTuple = struct.unpack('!d',data_bytes[8:16])
        AltitudeTuple = struct.unpack('!d',data_bytes[16:24])
        GPSTimeTuple = struct.unpack('!d',data_bytes[24:32])

        # store converted values 
        Longitude = LongitudeTuple[0]
        Latitude = LatitudeTuple[0]
        Altitude = AltitudeTuple[0]
        GPSTime = GPSTimeTuple[0]

        # Debug message 
        print(str(GPSTime) + "," + str(Longitude) + "," + str(Latitude) + "," + str(Altitude) + "\n")  

        synced = False  


socket_logger.close()
