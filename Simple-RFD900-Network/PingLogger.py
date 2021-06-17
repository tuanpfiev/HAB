# import libraries 
import time

# import files 
import GlobalVals
import NetworkManager
import CustMes

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
    NetworkManager.sendPacket(PingPacket)

    # intialise some variables 
    recievedPacket = CustMes.MESSAGE_FRAME()
    loopIndex = 0

    # wait for respnse from ping 
    while True:
        
        time.sleep(GlobalVals.PING_WAIT_TIME)

        # check if the thread needs to end 
        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            if GlobalVals.BREAK_PING_THREAD:
                return -2

        # if a packet hasn't been recieved yet
        if not GlobalVals.RECIEVED_PING:

            # check for time out and increment the loop counter 
            if loopIndex >= GlobalVals.PING_LOOP_LIMIT:
                print("PingLogger: Ping Timeout Error.")
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

#=====================================================
# Ping Logger Thread Function
#=====================================================
def PingLoggerThread():
    
    # get intial time to ping
    pingTime = time.time() + GlobalVals.PING_INTERVAL
    timeToPing = False

    # main thread loop
    while True:

        # check if the thread needs to end 
        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            if GlobalVals.BREAK_PING_THREAD:
                break
        
        # get the current time 
        currentTime = time.time()

        # check if it is time to ping
        if currentTime >= pingTime:
             timeToPing = True 
        else:
            continue
        
        # if it is time to ping 
        if timeToPing:
            
            # reset flag 
            timeToPing = False

            # intial values 
            pingTime = time.time() + GlobalVals.PING_INTERVAL
            pingStrings = []

            # get list of system IDs
            systemList = NetworkManager.RequestSystemsList()
            if len(systemList) == 0:
                continue
            
            # get ping for each system 
            # PingTarget is blocking 
            for x in systemList:
                ping = PingTarget(x)
                reportTime = int(time.time())
                if ping == -1:
                    pingStrings.append("Time: " + str(reportTime) + ", System " + str(x) + ", ERROR: Invalid system ID.\n")
                elif ping == -2:
                    pingStrings.append("Time: " + str(reportTime) + ", System " + str(x) + ", ERROR: Ping timed out.\n")
                else:
                    pingStrings.append("Time: " + str(reportTime) + ", System " + str(x) + ", Ping: " + str(ping) + "\n")

            # Open log file 
            try:
                fileObj = open(GlobalVals.PING_LOG_FILE, "a")
            except Exception as e:
                print("Exception: PingLogger: Open File: " + str(e.__class__)) 
                print("PingLoggerThread thread will now stop.")
                with GlobalVals.BREAK_PING_THREAD_MUTEX:
                    GlobalVals.BREAK_PING_THREAD = True
                break
            
            # Write to log file 
            try: 
                for x in pingStrings:
                    fileObj.write(x)
            except Exception as e:
                print("Exception: PingLogger: Write to File: " + str(e.__class__)) 
                print("PingLoggerThread thread will now stop.")
                with GlobalVals.BREAK_PING_THREAD_MUTEX:
                    GlobalVals.BREAK_PING_THREAD = True
                break
            finally:
                fileObj.close()
            

            

