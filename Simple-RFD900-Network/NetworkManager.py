# import libraries 
import time
import serial
from threading import Lock

# import files 
import GlobalVals
import CustMes


# a mutex for the sequence number function
SEQ_NUMBER_MUTEX = Lock()

#=====================================================
# Network Functions
#=====================================================

# Global Message increment function that rolls over at 8 bit limit
# should not be used outside of this thread!!!
def getSeqNumber():

    if GlobalVals.MESSAGE_ID == 255:
        GlobalVals.MESSAGE_ID = 0
    else:
        GlobalVals.MESSAGE_ID = GlobalVals.MESSAGE_ID + 1
    
    return GlobalVals.MESSAGE_ID

# function to send a packet (object)
def sendPacket(packetToSend):

    # put the packet in the output buffer 
    with GlobalVals.PACKET_BUFFER_OUT_MUTEX:
        GlobalVals.PACKET_BUFFER_OUT.append(packetToSend)

    
    # sent the send flag to true 
    with GlobalVals.SEND_PACKETS_MUTEX:
        GlobalVals.SEND_PACKETS = True
    
    return
                                                                                                                                               
# Function to clear the serial input buffer
def inputBufferClear():

    with GlobalVals.PACKET_BUFFER_IN_MUTEX:
        GlobalVals.PACKET_BUFFER_IN.clear()
    
    return 

# fuction for waiting and recieving a packet 
# MessageID - this is used to identify the type of message contained in the packet
# Newest - if true, the newest packet with a matching message id in the buffer will be returned otherwise the oldest is returned
# EraseOthers - if true, all packets with the same message id as the returned packet are removed from the buffer 
""" def getPacketOfType(MessageID, Newest, EraseOthers):
        
    timestamp = 0
    packetIndex = 0
    recievedPacket = CustMes.MESSAGE_FRAME()

    # if the oldest packet is wanted set the time stamp intially to the current time 
    if not Newest:
        timestamp = time.time()

    # lock the input buffer for use
    with GlobalVals.PACKET_BUFFER_IN_MUTEX:
        
        # get the number of packets in buffer 
        numPackets = len(GlobalVals.PACKET_BUFFER_IN)
        
        # If there are packets in the buffer
        if numPackets > 0:                
            x = 0    
            
            # using a while true since the length of the buffer can change while looking through it 
            while True:
                
                # if the packet has the correct message ID
                if GlobalVals.PACKET_BUFFER_IN[x].MessageID == MessageID:

                    # if the newest is needed 
                    if GlobalVals.PACKET_BUFFER_IN[x].Timestamp > timestamp and Newest:
                        timestamp = GlobalVals.PACKET_BUFFER_IN[x].Timestamp
                        packetIndex = x
                        x = x + 1

                        # pop the packet if erase others is enabled 
                        if EraseOthers:
                            recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(packetIndex)
                            x = x - 1
                    
                    # if the oldest is wanted 
                    elif GlobalVals.PACKET_BUFFER_IN[x].Timestamp < timestamp and not Newest:
                        timestamp = GlobalVals.PACKET_BUFFER_IN[x].Timestamp
                        packetIndex = x
                        x = x + 1

                        # pop the packet if erase others is enabled 
                        if EraseOthers:
                            recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(packetIndex)
                            x = x - 1
                else:
                    x = x + 1

                # if the index (x) exceeds the length of the list break the loop   
                if x >= len(GlobalVals.PACKET_BUFFER_IN):
                    break
            
            # if erase others isn't enabled pop the packet here 
            if not EraseOthers:
                recievedPacket = GlobalVals.PACKET_BUFFER_IN.pop(packetIndex)

        # if there are no packets return -1 
        else:
            return -1    
    
    # return the packet that was found 
    return recievedPacket
 """
# Ping Function
# This is blocking 
def PingTarget(TargetID):

    # check to see if the target is valid 
    # a broadcast ping is not allowed 
    if TargetID <= 0 or TargetID > 255:
        return -1

    # get required data classes 
    PingPacket = CustMes.MESSAGE_FRAME()
    PingData = CustMes.MESSAGE_PING()

    # set up message frame 
    PingPacket.MessageID = 1
    PingPacket.TargetID = TargetID
    PingPacket.SystemID = GlobalVals.SYSTEM_ID

    # set up payload 
    PingData.Intiator = True
    PingPacket.Payload = PingData.data_to_bytes()

    # send the ping 
    TimeSent = time.time()
    sendPacket(PingPacket)

    # intialise some variables 
    recievedPacket = CustMes.MESSAGE_FRAME()
    loopIndex = 0

    # wait for respnse from ping 
    while True:
        
        time.sleep(GlobalVals.PING_WAIT_TIME)

        # if a packet hasn't been recieved yet
        if not GlobalVals.RECIEVED_PING:

            # check for time out and increment the loop counter 
            if loopIndex >= GlobalVals.PING_LOOP_LIMIT:
                print("Network Manager: Ping Timeout Error.")
                return -2
            else:
                loopIndex = loopIndex + 1
                continue
        
        # if a packet has been recieved 
        else:

            # reset the flag
            with GlobalVals.RECIEVED_PING_MUTEX:
                GlobalVals.RECIEVED_PING = False
            
            # get the packet 
            with GlobalVals.PACKET_PING_BUFFER_MUTEX:
                found = False

                x = 0
                while x < len(GlobalVals.PACKET_PING_BUFFER):
                    if GlobalVals.PACKET_PING_BUFFER[x].SystemID == TargetID:

                        # remove old ping requests  
                        if GlobalVals.PACKET_PING_BUFFER[x].Timestamp < TimeSent:
                            GlobalVals.PACKET_PING_BUFFER.pop(x)
                            continue
                        
                        # keep only the newest packet 
                        else:
                            recievedPacket = GlobalVals.PACKET_PING_BUFFER.pop(x)
                            found = True
                    
                    x = x + 1

                # if a ping packet hasn't been recieved loop back
                if not found:
                    loopIndex = loopIndex + 1
                    continue

            break

    # get time stamp from packet and calculate the ping     
    TimeRec = recievedPacket.Timestamp
    ping = int((TimeRec - TimeSent) * 1000)

    # return the ping 
    return ping

def PingRespond(TargetID, TimeStamp):

    # check to see if the target is valid 
    # a broadcast ping is not allowed 
    if TargetID <= 0 or TargetID > 255:
        return -1

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

    # send the ping 
    sendPacket(PingPacket)

    return 0

# This function returns a list of known system IDs  
def RequestSystemsList():

    SystemsList = []

    with GlobalVals.SEQ_TRACKERS_MUTEX:
        numSystems = len(GlobalVals.SEQ_TRACKERS)
        for x in range(numSystems):
            SystemsList.append(GlobalVals.SEQ_TRACKERS[x].SystemID)
    
    return SystemsList

#=====================================================
# Network Manager Thread Function
#=====================================================
def SerialManagerThread():
    
    # Initialise variables 
    readBytes = bytearray()
    bufferRead = 1
    reading = True
    breakThread = False
    connected = True
    synced = False

    # Connect to the serial port 
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
    
    # Wait a second to let the port initialize
    time.sleep(1)

    # begin the loop used for reading and writing to the serial port 
    while connected and not breakThread:
        
        # check if the thread needs to read or send data 
        with GlobalVals.SEND_PACKETS_MUTEX:
            if not synced:
                reading = not GlobalVals.SEND_PACKETS
        
        # check if the thread needs to break 
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            breakThread = GlobalVals.BREAK_NETWORK_THREAD
        if breakThread:
            break 
        
        # if the thread needs to read data 
        if reading:

            # reset buffer read when not synced 
            if not synced:
                bufferRead = 1

            # read some bytes out of the buffer 
            try:
                comOut = serial_port.read(size=bufferRead)
            except serial.SerialTimeoutException:
                continue
            except (OSError, serial.SerialException) as e:
                print("Network Manager: Failed to read com port. Now breaking Thread.")
                print("Network Manager: Exception: " + str(e.__class__))
                breakThread = True
                continue
            
            if len(comOut) == 0:
                continue 
            
            # find the start bytes for the packet (0xFD) to sync the input buffer with the packet
            if not synced:
                readBytes.clear()
                if comOut[0] == 0xFD:
                    synced = True
                    readBytes.append(comOut[0])
                    continue
                else:
                    synced = False
                    continue 

            # Get the length of the packet once synced
            if synced and bufferRead == 1:
                bufferRead = comOut[0]          # get the payload length 
                bufferRead = bufferRead + 6     # (payload length + header bytes + CRC) - already read bytes 
                readBytes.append(comOut[0])
                continue 
            
            # form the rest of the packet 
            if bufferRead >= 2:
                
                # join the bytes that have already been read with the new ones 
                for x in comOut:
                    readBytes.append(x)

                # increment packet counter 
                with GlobalVals.PACKET_COUNT_MUTEX:
                    GlobalVals.PACKET_COUNT = GlobalVals.PACKET_COUNT + 1

                # Put the packet data into a message frame
                completePacket = CustMes.MESSAGE_FRAME()
                error = completePacket.bytes_to_data(readBytes)
                
                # if packet is corrupted  
                if error != 0:    
                    GlobalVals.reportError(1, error, 0)
                    print("Network Manager: Packet check error " + str(error) + " from " + str(completePacket.SystemID) + ". Getting next packet.\n")

                    # look for next packet 
                    synced = False
                    continue
                
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
                                print("Network Manager: Packet from " + str(completePacket.SystemID) + " is out of sequence by " + str(seqDiff) + "\n")
                                
                                # increment packet counter with missing packets  
                                with GlobalVals.PACKET_COUNT_MUTEX:
                                    GlobalVals.PACKET_COUNT = GlobalVals.PACKET_COUNT + seqDiff

                    else:
                        # if no trackers exist make a new one
                        IsNewTracker = True
                    
                    # if a new tracker needs to be made make one here 
                    if IsNewTracker:
                        newTracker = GlobalVals.sequenceTracker()
                        newTracker.SystemID = completePacket.SystemID
                        newTracker.CurrentNumber = completePacket.SeqNumber
                        GlobalVals.SEQ_TRACKERS.append(newTracker)

                # check if the packet is meant for this system 
                if completePacket.TargetID != GlobalVals.SYSTEM_ID and completePacket.TargetID != 0:
                    
                    # if it is not for this system reset values and look for next packet 
                    synced = False
                    readBytes.clear()
                    continue

                # timestamp packet 
                completePacket.Timestamp = time.time()

                # if the packet is a ping from an initiator, respond and discard packet 
                #if completePacket.MessageID == 1 and completePacket.Payload[0] == 0x00:
                #    PingRespond(completePacket.SystemID,completePacket.Timestamp)
                
                # if the packet is a ping response, put it in the ping buffer
                if completePacket.MessageID == 1 and completePacket.Payload[0] == 0xFF:
                    with GlobalVals.PACKET_PING_BUFFER_MUTEX:
                        while len(GlobalVals.PACKET_PING_BUFFER) >= GlobalVals.PACKET_BUFFER_IN_MAX_SIZE:
                            GlobalVals.PACKET_PING_BUFFER.pop(0)
                        GlobalVals.PACKET_PING_BUFFER.append(completePacket)

                        # set the recieved flag
                        with GlobalVals.RECIEVED_PING_MUTEX:
                            GlobalVals.RECIEVED_PING = True

                # Other packets are put into the standard packet buffer 
                else:

                    # debug check 
                    if completePacket.MessageID == 2 and len(completePacket.Payload) == 9:
                        print("FOUND IT!!!!") 

                    # append the packet to the input buffer and remove old packets
                    with GlobalVals.PACKET_BUFFER_IN_MUTEX:
                        while len(GlobalVals.PACKET_BUFFER_IN) >= GlobalVals.PACKET_BUFFER_IN_MAX_SIZE:
                            GlobalVals.PACKET_BUFFER_IN.pop(0)
                        GlobalVals.PACKET_BUFFER_IN.append(completePacket)
                    
                    # set recieved packet flag
                    with GlobalVals.RECIEVED_PACKETS_MUTEX:
                        GlobalVals.RECIEVED_PACKETS = True

                # reset values 
                synced = False
                readBytes.clear()
                continue
            
            # to catch if something happened out of order, in which case reset everything
            else:
                synced = False
                readBytes.clear()
                continue
        
        # if something needs to be written 
        else:
            
            # reset send flag 
            with GlobalVals.SEND_PACKETS_MUTEX:
                GlobalVals.SEND_PACKETS = False 

            #send each packet 
            with GlobalVals.PACKET_BUFFER_OUT_MUTEX: 
                while True:
                    if len(GlobalVals.PACKET_BUFFER_OUT) > 0:
                        
                        # get the next packet to send, set the sequence number and convert to bytes
                        outPacket = GlobalVals.PACKET_BUFFER_OUT.pop(0)
                        outPacket.SeqNumber = getSeqNumber()
                        bytesToSend = outPacket.data_to_bytes()

                        try:
                            serial_port.write(bytesToSend)
                            serial_port.flush()
                        except Exception as e:
                            print("Network Manager: Failed to write to com port. Now breaking thread.")
                            print("Network Manager: Exception: " + str(e.__class__))
                            breakThread = True
                            break
                    else:
                        break
    
    if connected:
        try:
            serial_port.close()
        except Exception as e:
            print("Network Manager: Exception: " + str(e.__class__))


    return