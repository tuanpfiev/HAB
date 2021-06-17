import binascii
from dataclasses import dataclass, field
from typing import List
import struct

#=====================================================
# CRC Globals
# DO NOT MODIFY!!!!!
#=====================================================
CRC_POLYNOMIAL = 0x1021     # the polynomial for the CRC calculation 
CRC_PRESET = 0              # preset for the CRC calculation 

#=====================================================
# Packet Globals (with no system label)
# DO NOT MODIFY!!!!!
#=====================================================
HEADER_BYTES = bytes(b'\x00')               # The sync header bytes 
HEADER_BYTES_SIZE = 1                       # the number of bytes for the sync header
CRC_SIZE = 2                                # the number of bytes for the CRC check
MESSAGE_FRAME_ARG_SIZE = 4                  # The number of bytes for the message frame arguments 
BROADCAST_ID = 0                            # the system ID for broadcasting to the network

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

    global MESSAGE_FRAME_ARG_SIZE
    global HEADER_BYTES_SIZE

    # get packet length 
    length = len(bytepacket)
    
    if length < (MESSAGE_FRAME_ARG_SIZE + HEADER_BYTES_SIZE):
        return -1 

    # perform CRC calculation and append it to the byte array 
    CRC_value = crcb(bytepacket[HEADER_BYTES_SIZE:length])
    CRC_bytes = CRC_value.to_bytes(CRC_SIZE, byteorder='big', signed=False)
    for i in range(CRC_SIZE):
        bytepacket.append(CRC_bytes[i])

    # return formated packet in byte array 
    return bytepacket

# this function checks the CRC, packet length, message ID, and message type and returns an error code if something is wrong
# error codes:
#   0  = no errors 
#   -1 = packet length is too small (<= 6 bytes)
#   -2 = length miss match between packet feild and measured length
#   -3 = CRC check failed 
def checkPacket(bytepacket):
    
    global MESSAGE_FRAME_ARG_SIZE
    global HEADER_BYTES_SIZE
    global CRC_SIZE

    test = bytepacket.hex()

    # Check length 
    packetLen = len(bytepacket)
    if packetLen < (MESSAGE_FRAME_ARG_SIZE + HEADER_BYTES_SIZE + CRC_SIZE):
        return -1

    # check reported length    
    inPacketLen = bytepacket[HEADER_BYTES_SIZE]
    if packetLen != (inPacketLen + MESSAGE_FRAME_ARG_SIZE + HEADER_BYTES_SIZE + CRC_SIZE):
        return -2

    # Check CRC
    bytepacket_crc = bytepacket[packetLen-CRC_SIZE:packetLen]
    bytepacket_cut = bytepacket[HEADER_BYTES_SIZE:packetLen-CRC_SIZE]
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
    TargetID: int = 0x00                                # 1 byte - ID 0 = reserved, ID 15 = broadcast
    MessageID: int = 0x00                               # 1 bytes
    Payload: bytearray = bytearray()                    # 0 - 255 bytes
    Timestamp: float = 0                                # not apart of the packet, used for timestamping when recieved
    ErrorCode: int = 0                                  # not apart of the packet, used to record an error in the packet

    def autoDataSize(self):
        self.PayloadLength = len(self.Payload)
    
    def data_to_bytes(self):

        global HEADER_BYTES

        # reset data size value 
        self.autoDataSize()

        # convert values into byte arrays
        PayloadLength_bytes = self.PayloadLength.to_bytes(1, byteorder='big', signed=False)
        SeqNumber_bytes = self.SeqNumber.to_bytes(1, byteorder='big', signed=False)
        SystemID_bytes = self.SystemID.to_bytes(1, byteorder='big', signed=False)
        TargetID_bytes = self.TargetID.to_bytes(1, byteorder='big', signed=False)
        MessageID_bytes = self.MessageID.to_bytes(1, byteorder='big', signed=False)

        # create byte array packet with values 
        bytePacketArray = bytearray()
        bytePacketArray_half = bytearray()

        for x in HEADER_BYTES:
            bytePacketArray.append(x)
        bytePacketArray.append(PayloadLength_bytes[0])
        bytePacketArray.append(SeqNumber_bytes[0])
        bytePacketArray.append(TargetID_bytes[0])
        bytePacketArray.append(MessageID_bytes[0])
                
        # append payload to the byte array 
        dataLength = self.PayloadLength
        if dataLength > 0:
            for i in range(dataLength):
                bytePacketArray.append(self.Payload[i])
        
        #claculate and append CRC
        bytePacketArray = append_CRC(bytePacketArray)
        if isinstance(bytePacketArray, int):
            self.ErrorCode = bytePacketArray
        
        # insert system ID into each byte including header
        # xxxxuuuu xxxxllll where 
        #  - xxxx is the system ID binary
        #  - uuuu is the upper nibble of the byte
        #  - llll is the lower nibble of the byte  
        for x in bytePacketArray:
            
            # get nibbles a and b from x
            a = (x & 0xf0) >> 4 
            b = (x & 0x0f)

            # include the system ID as the upper nibble 
            a = a | ((SystemID_bytes[0] & 0x0f) << 4)
            b = b | ((SystemID_bytes[0] & 0x0f) << 4)

            # append the bytes to the byte array 
            bytePacketArray_half.append(a)
            bytePacketArray_half.append(b)


        # return formated packet in byte array 
        return bytePacketArray_half
    
    def bytes_to_data(self, packetbytes):
        
        # get required globals 
        global HEADER_BYTES_SIZE
        global MESSAGE_FRAME_ARG_SIZE
        global HEADER_BYTES

        # formatedBytes will contain the packet with out the system ID labels 
        formatedBytes = bytearray()

        # check that all the bytes can be paired
        numBytes = len(packetbytes)
        remBytes = numBytes % 2
        if remBytes != 0:
            return -1

        # first byte to get system ID
        firstByte1 = packetbytes[0]
        firstByte2 = packetbytes[1]
        sysID1 = (firstByte1 & 0xf0) >> 4
        sysID2 = (firstByte2 & 0xf0) >> 4

        # if the system ID do not match return error -4
        if sysID1 != sysID2:
            return -4

        # save the system ID
        self.SystemID = sysID1
        
        # remove the system ID from the byte and save
        completeByte = 0
        completeByte |= (firstByte1 & 0x0f) << 4
        completeByte |= (firstByte2 & 0x0f)
        formatedBytes.append(completeByte)

        # each pair of bytes are split, the system ID removed and formed into a single byte
        for x in range(2,numBytes,2):
            
            byteA = packetbytes[x]
            byteB = packetbytes[x+1]

            sysID1 = (byteA & 0xf0) >> 4
            sysID2 = (byteB & 0xf0) >> 4

            # if the system IDs do not match then end
            if sysID1 != self.SystemID or sysID2 != self.SystemID:
                return -4 

            completeByte = 0
            completeByte |= (byteA & 0x0f) << 4
            completeByte |= (byteB & 0x0f)

            formatedBytes.append(completeByte)

        # Check packet for errors 
        self.ErrorCode = checkPacket(formatedBytes)

        # If there was an error return the error code
        if self.ErrorCode != 0:
            return self.ErrorCode
    
        # separate bytes into command feilds 
        self.PayloadLength = formatedBytes[HEADER_BYTES_SIZE]
        self.SeqNumber = formatedBytes[HEADER_BYTES_SIZE+1]
        self.TargetID = formatedBytes[HEADER_BYTES_SIZE+2]
        self.MessageID = formatedBytes[HEADER_BYTES_SIZE+3]
        
        self.Payload.clear()
        for x in range(self.PayloadLength):
            pos = x + MESSAGE_FRAME_ARG_SIZE + HEADER_BYTES_SIZE
            self.Payload.append(formatedBytes[pos])

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

# IMU Payload - Message ID = 0x04
@dataclass
class MESSAGE_IMU:
    Epoch: float = 0   
    Acceleration_i: float = 0 
    Acceleration_j: float = 0   
    Acceleration_k: float = 0   
    MagneticVector_i: float = 0
    MagneticVector_j: float = 0
    MagneticVector_k: float = 0
    RawQT_w: float = 0
    RawQT_i: float = 0
    RawQT_j: float = 0
    RawQT_k: float = 0
    Euler321_psi: float = 0
    Euler321_theta: float = 0
    Euler321_phi: float = 0
    Gyroscope_i: float = 0
    Gyroscope_j: float = 0
    Gyroscope_k: float = 0
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def setAcceleration(self,i,j,k):
        self.Acceleration_i = i
        self.Acceleration_j = j
        self.Acceleration_k = k
    
    def setMagneticVector(self,i,j,k):
        self.MagneticVector_i = i
        self.MagneticVector_j = j
        self.MagneticVector_k = k
        
    def setRawQT(self,w,i,j,k):
        self.RawQT_w = w
        self.RawQT_k = k
        self.RawQT_i = i
        self.RawQT_j = j
    
    def setEuler321(self,psi,theta,phi):
        self.Euler321_phi = phi
        self.Euler321_psi = psi
        self.Euler321_theta = theta
    
    def setGyroscope(self,i,j,k):
        self.Gyroscope_i = i
        self.Gyroscope_j = j
        self.Gyroscope_k = k

    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Epoch_ints = struct.pack('!d',self.Epoch)
        
        Acceleration_i_ints = struct.pack('!d',self.Acceleration_i)
        Acceleration_j_ints = struct.pack('!d',self.Acceleration_j)
        Acceleration_k_ints = struct.pack('!d',self.Acceleration_k)
        
        MagneticVector_i_ints = struct.pack('!d',self.MagneticVector_i)
        MagneticVector_j_ints = struct.pack('!d',self.MagneticVector_j)
        MagneticVector_k_ints = struct.pack('!d',self.MagneticVector_k)

        RawQT_i_ints = struct.pack('!d',self.RawQT_i)
        RawQT_j_ints = struct.pack('!d',self.RawQT_j)
        RawQT_k_ints = struct.pack('!d',self.RawQT_k)
        RawQT_w_ints = struct.pack('!d',self.RawQT_w)

        Euler321_phi_ints = struct.pack('!d',self.Euler321_phi)
        Euler321_theta_ints = struct.pack('!d',self.Euler321_theta)
        Euler321_psi_ints = struct.pack('!d',self.Euler321_psi)

        Gyroscope_i_ints = struct.pack('!d',self.Gyroscope_i)
        Gyroscope_j_ints = struct.pack('!d',self.Gyroscope_j)
        Gyroscope_k_ints = struct.pack('!d',self.Gyroscope_k)

        # append the values to the byte array for the payload 
        for x in Epoch_ints:
            payloadBytes.append(x)
        
        for x in Acceleration_i_ints:
            payloadBytes.append(x)

        for x in Acceleration_j_ints:
            payloadBytes.append(x)

        for x in Acceleration_k_ints:
            payloadBytes.append(x)

        for x in MagneticVector_i_ints:
            payloadBytes.append(x)

        for x in MagneticVector_j_ints:
            payloadBytes.append(x)

        for x in MagneticVector_k_ints:
            payloadBytes.append(x)
        
        for x in RawQT_i_ints:
            payloadBytes.append(x)

        for x in RawQT_j_ints:
            payloadBytes.append(x)

        for x in RawQT_k_ints:
            payloadBytes.append(x)

        for x in RawQT_w_ints:
            payloadBytes.append(x)

        for x in Euler321_phi_ints:
            payloadBytes.append(x)

        for x in Euler321_theta_ints:
            payloadBytes.append(x)

        for x in Euler321_psi_ints:
            payloadBytes.append(x)

        for x in Gyroscope_i_ints:
            payloadBytes.append(x)
        
        for x in Gyroscope_j_ints:
            payloadBytes.append(x)

        for x in Gyroscope_k_ints:
            payloadBytes.append(x)
        
        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 136:
            return -1
        
        # convert payload values back to double
        EpochTuple = struct.unpack('!d',payloadBytes[0:8])
        Acceleration_iTuple = struct.unpack('!d',payloadBytes[8:16])
        Acceleration_jTuple = struct.unpack('!d',payloadBytes[16:24])   
        Acceleration_kTuple = struct.unpack('!d',payloadBytes[24:32])
        MagneticVector_iTuple = struct.unpack('!d',payloadBytes[32:40])
        MagneticVector_jTuple = struct.unpack('!d',payloadBytes[40:48])
        MagneticVector_kTuple = struct.unpack('!d',payloadBytes[48:56])
        RawQT_iTuple = struct.unpack('!d',payloadBytes[56:64])
        RawQT_jTuple = struct.unpack('!d',payloadBytes[64:72])
        RawQT_kTuple = struct.unpack('!d',payloadBytes[72:80])
        RawQT_wTuple = struct.unpack('!d',payloadBytes[80:88])
        Euler321_phiTuple = struct.unpack('!d',payloadBytes[88:96])
        Euler321_thetaTuple = struct.unpack('!d',payloadBytes[96:104])
        Euler321_psiTuple = struct.unpack('!d',payloadBytes[104:112])
        Gyroscope_iTuple = struct.unpack('!d',payloadBytes[112:120])
        Gyroscope_jTuple = struct.unpack('!d',payloadBytes[120:128])
        Gyroscope_kTuple = struct.unpack('!d',payloadBytes[128:136])

        # store converted values 
        self.Epoch = EpochTuple[0] 
        self.Acceleration_i = Acceleration_iTuple[0] 
        self.Acceleration_j = Acceleration_jTuple[0]    
        self.Acceleration_k = Acceleration_kTuple[0] 
        self.MagneticVector_i = MagneticVector_iTuple[0] 
        self.MagneticVector_j = MagneticVector_jTuple[0] 
        self.MagneticVector_k = MagneticVector_kTuple[0] 
        self.RawQT_w = RawQT_wTuple[0]
        self.RawQT_i = RawQT_iTuple[0]
        self.RawQT_j = RawQT_jTuple[0] 
        self.RawQT_k = RawQT_kTuple[0] 
        self.Euler321_psi = Euler321_psiTuple[0] 
        self.Euler321_theta = Euler321_thetaTuple[0] 
        self.Euler321_phi = Euler321_phiTuple[0] 
        self.Gyroscope_i = Gyroscope_iTuple[0]
        self.Gyroscope_j = Gyroscope_jTuple[0]
        self.Gyroscope_k = Gyroscope_kTuple[0]

        return 0
# Temp Payload - Message ID = 0x05

@dataclass
class MESSAGE_TEMP:
    Epoch: float = 0   
    Temperature: float = 0 
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def setTemperature(self,temperature):
        self.Temperature = temperature
        

    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Epoch_ints = struct.pack('!d',self.Epoch)
        Temperature_ints = struct.pack('!d',self.Temperature)
        

        # append the values to the byte array for the payload 
        for x in Epoch_ints:
            payloadBytes.append(x)
        
        for x in Temperature_ints:
            payloadBytes.append(x)

        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 16:
            return -1
        
        # convert payload values back to double
        EpochTuple = struct.unpack('!d',payloadBytes[0:8])
        TemperatureTuple = struct.unpack('!d',payloadBytes[8:16])

        # store converted values 
        self.Epoch = EpochTuple[0] 
        self.Temperature = TemperatureTuple[0] 
        
        return 0

# RSSI Payload - Message ID = 0x06

@dataclass
class MESSAGE_RSSI:
    FilteredRSSI: float = 0   
    Epoch: float = 0 
    Distance: float = 0
    TargetPayloadID: float = 0
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def setFilteredRSSI(self,filteredRSSI):
        self.FilteredRSSI = filteredRSSI

    def setDistance(self,distance):
        self.Distance = distance

    def setTargetPayloadID(self,payloadID):
        self.TargetPayloadID = payloadID        

    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Epoch_ints = struct.pack('!d',self.Epoch)
        Distance_ints = struct.pack('!d',self.Distance)
        FilteredRSSI_ints = struct.pack('!d',self.FilteredRSSI)
        TargetPayloadID_ints = struct.pack('!d',self.TargetPayloadID)
        # append the values to the byte array for the payload 
        for x in Epoch_ints:
            payloadBytes.append(x)
        
        for x in Distance_ints:
            payloadBytes.append(x)

        for x in FilteredRSSI_ints:
            payloadBytes.append(x)

        for x in TargetPayloadID_ints:
            payloadBytes.append(x)

        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 32:
            return -1
        
        # convert payload values back to double
        EpochTuple = struct.unpack('!d',payloadBytes[0:8])
        DistanceTuple = struct.unpack('!d',payloadBytes[8:16])
        FilteredRSSITuple = struct.unpack('!d',payloadBytes[16:24])
        TargetPayloadIDTuple = struct.unpack('!d',payloadBytes[24:32])

        # print(".........")
        # print(EpochTuple)
        # print(DistanceTuple)
        # print(FilteredRSSITuple)
        # print(TargetPayloadIDTuple)

        # store converted values 
        self.Epoch = EpochTuple[0] 
        self.Distance = DistanceTuple[0] 
        self.FilteredRSSI = FilteredRSSITuple[0]
        self.TargetPayloadID = TargetPayloadIDTuple[0]
        return 0



@dataclass
class MESSAGE_RSSI_ALLOCATION:
    Pair: float = 0   
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def setPair(self,pair):
        self.Pair = pair

    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Pair_ints = struct.pack('!d',self.Pair)
                
        # append the values to the byte array for the payload 
        for x in Pair_ints:
            payloadBytes.append(x)
        
        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 8:
            return -1
        
        # convert payload values back to double
        PairTuple = struct.unpack('!d',payloadBytes[0:8])
        # store converted values 
        self.Pair = PairTuple[0] 
        return 0


@dataclass
class MESSAGE_RSSI:
    FilteredRSSI: float = 0   
    Epoch: float = 0 
    Distance: float = 0
    TargetPayloadID: float = 0
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def setFilteredRSSI(self,filteredRSSI):
        self.FilteredRSSI = filteredRSSI

    def setDistance(self,distance):
        self.Distance = distance

    def setTargetPayloadID(self,payloadID):
        self.TargetPayloadID = payloadID        

    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Epoch_ints = struct.pack('!d',self.Epoch)
        Distance_ints = struct.pack('!d',self.Distance)
        FilteredRSSI_ints = struct.pack('!d',self.FilteredRSSI)
        TargetPayloadID_ints = struct.pack('!d',self.TargetPayloadID)
        # append the values to the byte array for the payload 
        for x in Epoch_ints:
            payloadBytes.append(x)
        
        for x in Distance_ints:
            payloadBytes.append(x)

        for x in FilteredRSSI_ints:
            payloadBytes.append(x)

        for x in TargetPayloadID_ints:
            payloadBytes.append(x)

        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 32:
            return -1
        
        # convert payload values back to double
        EpochTuple = struct.unpack('!d',payloadBytes[0:8])
        DistanceTuple = struct.unpack('!d',payloadBytes[8:16])
        FilteredRSSITuple = struct.unpack('!d',payloadBytes[16:24])
        TargetPayloadIDTuple = struct.unpack('!d',payloadBytes[24:32])

        # print(".........")
        # print(EpochTuple)
        # print(DistanceTuple)
        # print(FilteredRSSITuple)
        # print(TargetPayloadIDTuple)

        # store converted values 
        self.Epoch = EpochTuple[0] 
        self.Distance = DistanceTuple[0] 
        self.FilteredRSSI = FilteredRSSITuple[0]
        self.TargetPayloadID = TargetPayloadIDTuple[0]
        return 0


# EKF Payload - Message ID = 0x09
@dataclass
class MESSAGE_EKF:
    Epoch: float = 0   
    Latitude: float = 0 
    Longitude: float = 0   
    Altitude: float = 0   
    PosX: float = 0
    PosY: float = 0
    P00: float = 0
    P01: float = 0
    P10: float = 0
    P11: float = 0
    SystemID: int = 0       # This isn't apart of the payload structure, it is used to just keep track of where the data came from  
    
    def data_to_bytes(self):

        # create the payload byte array 
        payloadBytes = bytearray()

        # split values into 8 bit ints 
        Epoch_ints = struct.pack('!d',self.Epoch)
        
        Latitude_ints = struct.pack('!d',self.Latitude)
        Longitude_ints = struct.pack('!d',self.Longitude)
        Altitude_ints = struct.pack('!d',self.Altitude)

        PosX_ints = struct.pack('!d',self.PosX)
        PosY_ints = struct.pack('!d',self.PosY)
        P00_ints = struct.pack('!d',self.P00)
        P01_ints = struct.pack('!d',self.P01)
        P10_ints = struct.pack('!d',self.P10)
        P11_ints = struct.pack('!d',self.P11)
        # append the values to the byte array for the payload 
        for x in Epoch_ints:
            payloadBytes.append(x)
        
        for x in Latitude_ints:
            payloadBytes.append(x)

        for x in Longitude_ints:
            payloadBytes.append(x)

        for x in Altitude_ints:
            payloadBytes.append(x)

        for x in PosX_ints:
            payloadBytes.append(x)

        for x in PosY_ints:
            payloadBytes.append(x)

        for x in P00_ints:
            payloadBytes.append(x)
        
        for x in P01_ints:
            payloadBytes.append(x)
        
        for x in P10_ints:
            payloadBytes.append(x)

        for x in P11_ints:
            payloadBytes.append(x)

        # return the payload byte array 
        return payloadBytes

    def bytes_to_data(self, payloadBytes):

        # check length of payload 
        payloadLen = len(payloadBytes)
        if payloadLen != 80:
            return -1
        
        # convert payload values back to double
        EpochTuple = struct.unpack('!d',payloadBytes[0:8])
        LatitudeTuple = struct.unpack('!d',payloadBytes[8:16])
        LongitudeTuple = struct.unpack('!d',payloadBytes[16:24])   
        AltitudeTuple = struct.unpack('!d',payloadBytes[24:32])
        PosXTuple = struct.unpack('!d',payloadBytes[32:40])
        PosYTuple = struct.unpack('!d',payloadBytes[40:48])
        P00Tuple = struct.unpack('!d',payloadBytes[48:56])
        P01Tuple = struct.unpack('!d',payloadBytes[56:64])
        P10Tuple = struct.unpack('!d',payloadBytes[64:72])
        P11Tuple = struct.unpack('!d',payloadBytes[72:80])

        # store converted values 
        self.Epoch = EpochTuple[0] 
        self.Latitude = LatitudeTuple[0] 
        self.Longitude = LongitudeTuple[0]    
        self.Altitude = AltitudeTuple[0] 
        self.PosX = PosXTuple[0] 
        self.PosY = PosYTuple[0] 
        self.P00 = P00Tuple[0] 
        self.P01 = P01Tuple[0]
        self.P10 = P10Tuple[0] 
        self.P11 = P11Tuple[0] 

        return 0