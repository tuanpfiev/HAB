# import libraries 
import time
import serial
from threading import Lock
from dataclasses import dataclass

# import files 
import GlobalVals
import CustMes

# a mutex for the sequence number function
SEQ_NUMBER_MUTEX = Lock()

# Debugging function - byte dump  
# saves a copy of the bytes in debugBytes
def ByteDump(debugBytes, marker):
    
    logString = marker + debugBytes.hex() + '\n'
    #debugBytes.clear()
    try:
        fileObj = open("SerialDump_" + str(GlobalVals.SYSTEM_ID) + ".txt", "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: NetworkManager: Debug Log File Write: " + str(e.__class__))
        print("NetworkManager thread will continue.")

#=====================================================
# Byte Bucket Class
#=====================================================
class ByteBucket:

    #************************************
    # class intialisation 
    # input:
    #   - IntSystemID - the system ID of the system this bucket will collect bytes for
    #************************************
    def __init__(self, IntSystemID):
        self.__Bucket = bytearray()
        self.__SystemID = IntSystemID

        self.__checkHeader = bytearray()
        for x in CustMes.HEADER_BYTES:
            h1 = ((x & 0xF0) >> 4) | ((self.__SystemID & 0x0F) << 4)
            h2 = (x & 0x0F) | ((self.__SystemID & 0x0F) << 4)
            self.__checkHeader.append(h1)
            self.__checkHeader.append(h2)
        
        self.__numHeaderBytes = len(self.__checkHeader)
        self.__synced = False
        self.__syncs = [False] * self.__numHeaderBytes
        self.__syncPos = 0
        self.__curState = 0
        self.__remBytes = 0
        self.__tempBytes = bytearray()
        self.__PacketComplete = False
    
    #************************************
    # function for reseting all internal values 
    #************************************
    def resetBucket(self):
        
        for x in range(self.__numHeaderBytes):
            self.__syncs[x] = False

        self.__Bucket.clear()
        self.__synced = False
        self.__syncPos = 0
        self.__curState = 0
        self.__remBytes = 0
        self.__tempBytes.clear()
        self.__PacketComplete = False

        return
    
    #************************************
    # function for adding and checking a byte to put in the bucket
    # Input:
    #   - Byte - This is a byte object that will be checked and added to the byte bucket
    # Output 
    #   -  0    - everything when fine 
    #   - -1    - a packet has already been completed so this byte was discared
    #   - -2    - the byte has a bad system ID and was discarded
    #************************************
    def addByte(self, Byte):

        # states:
        #   -  0    = finding the sync header 
        #   -  1    = finding the length of the packet 
        #   -  2    = getting the rest of the packet  

        # if the packet is complete do not accept a byte
        if self.__PacketComplete:
            return -1 
        
        # if still syncing 
        if self.__synced == False:

            self.__Bucket.clear()
            self.__tempBytes.clear()

            # reset syncpos if out of range
            if self.__syncPos >= self.__numHeaderBytes:
                for x in range(self.__numHeaderBytes):
                    self.__syncs[x] = False
                self.__syncPos = 0

            # if the syncPos th header byte is the current byte being read mark it and move on
            if Byte == self.__checkHeader[self.__syncPos]:
                self.__syncs[self.__syncPos] = True
                self.__syncPos = self.__syncPos + 1    
            
            # if it isn't the current byte 
            else:
                
                # reset sync values 
                for x in range(self.__numHeaderBytes):
                    self.__syncs[x] = False
                self.__syncPos = 0
                
                # in case it is the start of a header check the first byte
                if Byte == self.__checkHeader[0]:
                    self.__syncs[0] = True
                    self.__syncPos = 1
        
            # if all the sync header bytes have been found 
            if all(self.__syncs):
                self.__synced = True
                
                # reset sync values 
                for x in range(self.__numHeaderBytes):
                    self.__syncs[x] = False
                self.__syncPos = 0

        # this is the syncing state   
        if self.__curState == 0:

            # if synced loaded the sync header into bucket
            if self.__synced:
                self.__curState = 1
                for x in self.__checkHeader:
                    self.__Bucket.append(x)
            
            return 0
        
        # byte ID check
        # bad ID = -2
        sysID = (Byte & 0xF0) >> 4
        if sysID != self.__SystemID:
            self.resetBucket = False
            return -2
        
        # this is the size state 
        if self.__curState == 1:

            # wait for 2 bytes 
            self.__tempBytes.append(Byte)
            if (len(self.__tempBytes) == 2):
                
                # remove sysID and join the two bytes to get the length 
                a = (self.__tempBytes[0] & 0x0F) << 4
                b = self.__tempBytes[1] & 0x0F
                c = a | b
                for x in self.__tempBytes:
                    self.__Bucket.append(x)
                self.__tempBytes.clear()

                # calculate the remaining bytes 
                self.__remBytes = c + (CustMes.MESSAGE_FRAME_ARG_SIZE - 1) + CustMes.CRC_SIZE
                self.__remBytes = self.__remBytes * 2

                # change state 
                self.__curState = 2
            
            return 0
        
        # this state gets the remaining bytes 
        if self.__curState == 2:
            
            # add byte to bucket
            self.__Bucket.append(Byte)
            self.__remBytes = self.__remBytes - 1

            # if all remaining bytes have been collected reset some values
            if self.__remBytes < 1:
                self.__remBytes = 0
                self.__curState = 0
                self.__synced = False
                self.__PacketComplete = True
            
            return 0
    
    #************************************
    # function to find out if a packet is complete
    # Output:
    #   - True  - packet is complete
    #   - False - packet is not complete 
    #************************************
    def isPacketComplete(self):
        return self.__PacketComplete
    
    #************************************
    # function for returning the system ID
    # Output:
    #   - systemID (int) - the system ID for this bucket
    #************************************
    def getID(self):
        return self.__SystemID
    
    #************************************
    # function to get completed packet
    # Output:
    #   - 0             - packet is not complete
    #   - bytearray()   - the completed packet
    #************************************
    def getPacket(self):

        if not self.__PacketComplete:
            return 0
        
        self.__PacketComplete = False
        tempArray = self.__Bucket.copy()
        self.resetBucket()

        return tempArray


# Byte Bucket List 
ByteBuckets = []

#=====================================================
# Network Functions
#=====================================================

#************************************
# Function for safely retriving the sequence number 
# should not be used outside of this thread!!!
# Output:
#   - sequence number - the next sequence number to use
#************************************
def getSeqNumber():

    if GlobalVals.SEQUENCE_NUM == 255:
        GlobalVals.SEQUENCE_NUM = 0
    else:
        GlobalVals.SEQUENCE_NUM = GlobalVals.SEQUENCE_NUM + 1
    
    return GlobalVals.SEQUENCE_NUM


#************************************
# Function for safely adding packets to the send buffer. 
#************************************
def sendPacket(packetToSend):

    # put the packet in the output buffer 
    with GlobalVals.PACKET_BUFFER_OUT_MUTEX:
        GlobalVals.PACKET_BUFFER_OUT.append(packetToSend)
    
    # set the send flag to true 
    with GlobalVals.SEND_PACKETS_MUTEX:
        GlobalVals.SEND_PACKETS = True
    
    return

#************************************
# Function for clearing the packet input buffer 
#************************************
def inputBufferClear():

    with GlobalVals.PACKET_BUFFER_IN_MUTEX:
        for i in GlobalVals.PACKET_BUFFER_IN:
            i.clear()
    
    return 

#************************************
# Function for responding to a ping 
# Input:
#   - TargetID  - The system ID of the system that sent the ping 
#   - TimeStamp - the time stamp of the recieved ping packet 
# Output:
#   -  0 - no problem 
#   - -1 - TargetID is invalid 
#************************************
def PingRespond(TargetID, TimeStamp):

    # a broadcast ping is not allowed 
    if TargetID == CustMes.BROADCAST_ID:
        return -1
    
    # keep only the lower 4 bits for the ID
    TargetID = TargetID & 0x0F

    # get required data classes 
    PingPacket = CustMes.MESSAGE_FRAME()
    PingData = CustMes.MESSAGE_PING()

    # set up message frame 
    PingPacket.MessageID = 1
    PingPacket.TargetID = TargetID
    PingPacket.SystemID = GlobalVals.SYSTEM_ID

    # set up payload 
    PingData.Intiator = False
    PingData.TimeDiff = time.time() - TimeStamp
    PingPacket.Payload = PingData.data_to_bytes()

    # put the packet in the output buffer 
    with GlobalVals.PACKET_BUFFER_OUT_MUTEX:
        GlobalVals.PACKET_BUFFER_OUT.append(PingPacket)
    
    # set the send flag to true 
    with GlobalVals.SEND_PACKETS_MUTEX:
        GlobalVals.SEND_PACKETS = True

    return 0

#************************************
# Function for retrieving a list of known systems 
# Output:
#   - SystemsList - a list of system IDs (ints) of all known systems
#************************************
def RequestSystemsList():

    SystemsList = []

    with GlobalVals.SEQ_TRACKERS_MUTEX:
        numSystems = len(GlobalVals.SEQ_TRACKERS)
        for x in range(numSystems):
            SystemsList.append(GlobalVals.SEQ_TRACKERS[x].SystemID)
    
    return SystemsList

#************************************
# This function processes the bytes of complete packets into useable data.
# the processed packet is then appended to the packet buffer for use with other scripts.
# Input:
#   - readBytes (bytearray) - this byte array contaons the bytes of a completed packet
#************************************
def ByteProcessing(readBytes):

    # debugging - byte dump
    #ByteDump(readBytes,'ByteProcessing ')

    # increment packet counter 
    with GlobalVals.PACKET_COUNT_MUTEX:
        GlobalVals.PACKET_COUNT = GlobalVals.PACKET_COUNT + 1

    # Put the packet data into a message frame
    completePacket = CustMes.MESSAGE_FRAME()
    error = completePacket.bytes_to_data(readBytes)
    
    # if packet has an error in converting   
    if error != 0:    
        GlobalVals.reportError(1, error, 0)
        print("Packet Error: NetworkManager: Packet check error " + str(error) + " from " + str(completePacket.SystemID) + ". Getting next packet.")
        # print(readBytes.hex())
        return
    # print("packet ok: ",readBytes.hex())
    # update sequence trackers 
    with GlobalVals.SEQ_TRACKERS_MUTEX:
        
        TrackerLength = len(GlobalVals.SEQ_TRACKERS)
        IsNewTracker = True
        
        # if trackers exist 
        if TrackerLength > 0:
            
            # find a tracker with a matching SystemID
            trackerIndex = 0
            for x in range(TrackerLength):
                if GlobalVals.SEQ_TRACKERS[x].SystemID == completePacket.SystemID:
                    IsNewTracker = False
                    trackerIndex = x
                    break
            
            # if one does exist update the tarcker 
            if not IsNewTracker:
                
                seqDiff = GlobalVals.SEQ_TRACKERS[trackerIndex].NextNumber(completePacket.SeqNumber)
                
                # if the sequence has broken report the error
                if seqDiff != 0:
                    GlobalVals.reportError(3, seqDiff, completePacket.SystemID)
                    print("Sequence Error: NetworkManager: Packet from " + str(completePacket.SystemID) + " is out of sequence by " + str(seqDiff) + "\n")
                    # if seqDiff != 256:
                    #     print("seqDiff: ",seqDiff)

                    # increment packet counter with missing packets  
                    with GlobalVals.PACKET_COUNT_MUTEX:
                        GlobalVals.PACKET_COUNT = GlobalVals.PACKET_COUNT + seqDiff
                        # print("packet_count: ",GlobalVals.PACKET_COUNT)

        else:
            # if no trackers exist make a new one
            IsNewTracker = True
        
        # if a new tracker needs to be made make one here 
        if IsNewTracker:
            newTracker = GlobalVals.sequenceTracker()
            newTracker.SystemID = completePacket.SystemID
            newTracker.CurrentNumber = completePacket.SeqNumber
            GlobalVals.SEQ_TRACKERS.append(newTracker)

    # if this packet is not meant for this system, return 
    if completePacket.TargetID != GlobalVals.SYSTEM_ID and completePacket.TargetID != CustMes.BROADCAST_ID:
        return

    # timestamp packet 
    completePacket.Timestamp = time.time()

    # append the packet to the input buffer and remove old packets
    with GlobalVals.PACKET_BUFFER_IN_MUTEX:
        GlobalVals.PACKET_BUFFER_IN[completePacket.MessageID].append(completePacket)
    
    # set recieved packet flag
    with GlobalVals.RECIEVED_PACKETS_MUTEX:
        GlobalVals.RECIEVED_PACKETS = True
    
    return 


#=====================================================
# RFD900 Manager Thread Function
#=====================================================
def RFD900_ManagerThread():

    # Initialise variables 
    reading = True
    byteID = 0
    bucketPos = 0

    # include globals 
    global ByteBuckets
 
    # Connect to the serial port 
    try:
        serial_port = serial.Serial(
            port=GlobalVals.PORT,
            baudrate=GlobalVals.BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=GlobalVals.TIMEOUT
        )
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
    except Exception as e:
        print("Exception: NetworkManager: Serial Port Connection: " + str(e.__class__))
        print("NetworkManager thread will now stop." )
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            GlobalVals.BREAK_NETWORK_THREAD = True
        return

    # begin the loop used for reading and writing to the serial port 
    while True:

        # check if the thread needs to read or send data 
        with GlobalVals.SEND_PACKETS_MUTEX:
            reading = not GlobalVals.SEND_PACKETS
        
        # check if the thread needs to break 
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            if GlobalVals.BREAK_NETWORK_THREAD:
                break 
        
        #************************************
        # Serial Read
        #************************************
        if reading:

            # read serial buffer 
            try:
                # bytesToRead = serial_port.inWaiting()
                # comOut = serial_port.read(bytesToRead)
                comOut = serial_port.read(size=128)
            except serial.SerialTimeoutException:
                continue
            except (OSError, serial.SerialException) as e:
                print("Exception: NetworkManager: Read Serial Port: " + str(e.__class__))
                print("NetworkManager thread will now stop.")
                with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
                    GlobalVals.BREAK_NETWORK_THREAD = True
                break
            
            # If the buffer is empty try again
            if len(comOut) == 0:
                continue 
            
            # debuging - byte dump
            # else:
            #    ByteDump(comOut,'Buffer ')

            # go through each byte of the buffer 
            for comByte in comOut:
            
                # determine the byte ID
                byteID = (comByte & 0xF0) >> 4
                
                # check if ID matches any of the buckets 
                bucketPos = -1
                for x in range(len(ByteBuckets)):
                    if ByteBuckets[x].getID() == byteID:
                        bucketPos = x
                        break

                # if there are no matches make a new bucket
                if bucketPos == -1:
                    newBucket = ByteBucket(byteID)
                    ByteBuckets.append(newBucket)
                    bucketPos = len(ByteBuckets) - 1

                # add byte to bucket
                error = ByteBuckets[bucketPos].addByte(comByte)
                
                # if bucket has a complete packet, process it and then add the byte
                if error == -1:
                    ByteProcessing(ByteBuckets[bucketPos].getPacket())
                    error = ByteBuckets[bucketPos].addByte(comByte)

                #if some other erro occurs report it and continue 
                if error != 0:
                    print("NetworkManager: Packet Error: " + str(error))
                    print("Byte will be ignored.")
                    continue

                # check all buckets and process any complete packets
                for x in range(len(ByteBuckets)):
                    if ByteBuckets[x].isPacketComplete():
                        ByteProcessing(ByteBuckets[x].getPacket())
            
            
        #************************************
        # Serial Write 
        #************************************
        else:

            #send each packet 
            with GlobalVals.PACKET_BUFFER_OUT_MUTEX: 
                
                #while len(GlobalVals.PACKET_BUFFER_OUT) > 0:
                if len(GlobalVals.PACKET_BUFFER_OUT) != 0:

                    # get the next packet to send, set the sequence number and convert to bytes
                    outPacket = GlobalVals.PACKET_BUFFER_OUT.pop(0)
                    outPacket.SeqNumber = getSeqNumber()
                    bytesToSend = outPacket.data_to_bytes()

                    try:
                        serial_port.write(bytesToSend)
                        serial_port.flush()
                    except Exception as e:
                        print("Exception: NetworkManager: Writting to Serial Port: " + str(e.__class__))
                        print("NetworkManager thread will now stop.")
                        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
                            GlobalVals.BREAK_NETWORK_THREAD = True
                        break
                        
                    timeLastSent = time.time()
                
                else:

                    # reset send flag 
                    with GlobalVals.SEND_PACKETS_MUTEX:
                        GlobalVals.SEND_PACKETS = False 
    
    
    try:
        serial_port.close()
    except Exception as e:
        print("Network Manager: Exception: " + str(e.__class__))


    return