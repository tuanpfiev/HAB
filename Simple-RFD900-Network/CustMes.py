import binascii
from dataclasses import dataclass, field
from typing import List
import struct

# CRC globals 
CRC_POLYNOMIAL = 0x1021     # the polynomial for the CRC calculation 
CRC_PRESET = 0              # preset for the CRC calculation 

#=====================================================
# CRC Calculation Functions 
#=====================================================
def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ CRC_POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff

    return crc

def crc(str):
    crc = CRC_PRESET
    for c in str:
        crc = _update_crc(crc, ord(c))
    return crc

def crcb(i):
    crc = CRC_PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc

# this function calculates the CRC of the packet and appends it to the packet
def append_CRC(bytepacket):

    # get packet length 
    length = len(bytepacket)
    
    if length < 7:
        return -1 

    # perform CRC calculation and append it to the byte array 
    CRC_value = crcb(bytepacket[1:length])
    CRC_bytes = CRC_value.to_bytes(2, byteorder='big', signed=False)
    bytepacket.append(CRC_bytes[0])
    bytepacket.append(CRC_bytes[1])

    # return formated packet in byte array 
    return bytepacket

# this function checks the CRC, packet length, message ID, and message type and returns an error code if something is wrong
# error codes:
#   0  = no errors 
#   -1 = packet length is too small (<= 6 bytes)
#   -2 = length miss match between packet feild and measured length
#   -3 = CRC check failed 
def checkPacket(bytepacket):
    
    # Check length 
    packetLen = len(bytepacket)
    if packetLen < 8:  # a minimum of 8 bytes is needed for the prefix and sufix bytes
        return -1

    # check reported length    
    inPacketLen = bytepacket[1]
    if packetLen != (inPacketLen + 8):
        return -2

    # Check CRC
    bytepacket_crc = bytepacket[packetLen-2:packetLen]
    bytepacket_cut = bytepacket[1:packetLen-2]
    packetCRC = crcb(bytepacket_cut)
    inPacketCRC = int.from_bytes(bytepacket_crc, byteorder='big', signed=False)
    if inPacketCRC != packetCRC:
        return -3
    
    return 0

#=====================================================
# Message frame data classes
#=====================================================

@dataclass
class MESSAGE_FRAME:
    PayloadLength: int = 0x00                           # 1 byte
    SeqNumber: int = 0x00                               # 1 byte
    SystemID: int = 0x00                                # 1 byte
    TargetID: int = 0x00                                # 1 byte - ID 0 is a braodcast
    MessageID: int = 0x00                               # 1 bytes
    Payload: bytearray = bytearray()                    # 0 - 255 bytes
    Timestamp: float = 0                                # not apart of the packet, used for timestamping when recieved
    ErrorCode: int = 0

    def autoDataSize(self):
        self.PayloadLength = len(self.Payload)
    
    def data_to_bytes(self):

        # reset data size value 
        self.autoDataSize()

        # convert values into byte arrays
        PayloadLength_bytes = self.PayloadLength.to_bytes(1, byteorder='big', signed=False)
        SeqNumber_bytes = self.SeqNumber.to_bytes(1, byteorder='big', signed=False)
        SystemID_bytes = self.SystemID.to_bytes(1, byteorder='big', signed=False)
        TargetID_bytes = self.TargetID.to_bytes(1, byteorder='big', signed=False)
        MessageID_bytes = self.MessageID.to_bytes(1, byteorder='big', signed=False)

        # create byte array packet with values 
        bytePacketArray = bytearray(b'\xFD')                # 00 and 01 - sync header 
        bytePacketArray.append(PayloadLength_bytes[0])
        bytePacketArray.append(SeqNumber_bytes[0])
        bytePacketArray.append(SystemID_bytes[0])
        bytePacketArray.append(TargetID_bytes[0])
        bytePacketArray.append(MessageID_bytes[0])
                
        # append payload to the byte array 
        dataLength = self.PayloadLength
        if dataLength > 0:
            for i in range(dataLength):
                bytePacketArray.append(self.Payload[i])
        
        #claculate CRC and Length
        bytePacketArray = append_CRC(bytePacketArray)
        if isinstance(bytePacketArray, int):
            self.ErrorCode = bytePacketArray

        # return formated packet in byte array 
        return bytePacketArray
    
    def bytes_to_data(self, packetbytes):
    
        # Check packet for errors 
        self.ErrorCode = checkPacket(packetbytes)

        # If there was an error return the error code
        if self.ErrorCode != 0:
            return self.ErrorCode
    
        # separate bytes into command feilds 
        self.PayloadLength = packetbytes[1]
        self.SeqNumber = packetbytes[2]
        self.SystemID = packetbytes[3]
        self.TargetID = packetbytes[4]
        self.MessageID = packetbytes[5]
        
        self.Payload.clear()
        for x in range(self.PayloadLength):
            pos = x + 6
            self.Payload.append(packetbytes[pos])

        # Return error code 0 confirming there were no issues
        self.ErrorCode = 0
        return 0

#=====================================================
# Payload data classes
#=====================================================

# Ping Payload - Message ID = 0x01
@dataclass
class MESSAGE_PING:
    Intiator: bool = True   # 1 byte, 0x00 = intiator, 0xFF = response
    TimeDiff: float = 0     # 8 bytes, empty for intiator, time diff for response 
    
    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # Setup ping type 
        if self.Intiator:
            payloadBytes.append(0x00)
            self.TimeDiff = 0
        else:
            payloadBytes.append(0xFF)

        # split timediff into 8 ints 
        TimeDiff_ints = struct.pack('!d',self.TimeDiff)

        # put the time diff into the payload 
        for x in TimeDiff_ints:
            payloadBytes.append(x)
        
        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 9:
            return -1
        
        # set intiator flag
        if payloadBytes[0] == 0x00:
            self.Intiator = True
        else:
            self.Intiator = False
        
        # convert timediff back to double
        TimeTuple = struct.unpack('!d',payloadBytes[1:9])
        self.TimeDiff = TimeTuple[0]

        return 0

# GPS Payload - Message ID = 0x02
@dataclass
class MESSAGE_GPS:
    Longitude: float = 0   
    Latitude: float = 0     
    Altitude: float = 0
    GPSTime: float = 0
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Longitude_ints = struct.pack('!d',self.Longitude)
        Latitude_ints = struct.pack('!d',self.Latitude)
        Altitude_ints = struct.pack('!d',self.Altitude)
        GPSTime_ints = struct.pack('!d',self.GPSTime)

        # append the values to the byte array for the payload 
        for x in Longitude_ints:
            payloadBytes.append(x)
        
        for x in Latitude_ints:
            payloadBytes.append(x)

        for x in Altitude_ints:
            payloadBytes.append(x)

        for x in GPSTime_ints:
            payloadBytes.append(x)
        
        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 32:
            return -1
        
        # convert payload values back to double
        LongitudeTuple = struct.unpack('!d',payloadBytes[0:8])
        LatitudeTuple = struct.unpack('!d',payloadBytes[8:16])
        AltitudeTuple = struct.unpack('!d',payloadBytes[16:24])
        GPSTimeTuple = struct.unpack('!d',payloadBytes[24:32])

        # store converted values 
        self.Longitude = LongitudeTuple[0]
        self.Latitude = LatitudeTuple[0]
        self.Altitude = AltitudeTuple[0]
        self.GPSTime = GPSTimeTuple[0]

        return 0

# GPS Payload - Message ID = 0x03
@dataclass
class MESSAGE_STR:
    MessageStr: str = ""

    def data_to_bytes(self):

        # create the payload byte array and return it  
        payloadBytes = bytearray(self.MessageStr, 'utf-8')
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # decode byte array and store it 
        try:
            self.MessageStr = payloadBytes.decode('utf-8')
        except Exception:
            # if an error occurs return -1 
            return -1
        
        # return zero indicating there was no problem 
        return 0
