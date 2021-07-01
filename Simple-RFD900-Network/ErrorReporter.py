# import libraries 
from ctypes import RTLD_GLOBAL
from os import error
import time

# import files 
import GlobalVals

#=====================================================
# Error Logger Thread Function
#=====================================================
def ErrorLogger():

    # set up time markers 
    curTime = time.time()
    statTime = curTime + GlobalVals.PACKET_STATS_INTERVAL

    # setup count v
    errorTypes = [[] for _ in range(GlobalVals.N_REAL_BALLOON+1)]
    errorTypeCount = [[] for _ in range(GlobalVals.N_REAL_BALLOON+1)]

    # loop for checking errors 
    while True:

        # check error detected flag to see if there are errors 
        Errors = False
        with GlobalVals.ERROR_DETECTED_MUTEX:
            Errors = GlobalVals.ERROR_DETECTED
        
        # if there are no errors sleep for 0.5 secs
        if not Errors:
            time.sleep(0.5)
        
        # Loop through all the error records 
        with GlobalVals.PACKET_ERROR_MUTEX:
            while len(GlobalVals.PACKET_ERROR_QUE) > 0:
                
                # pop and make a log string from the report 
                CurError = GlobalVals.PACKET_ERROR_QUE.pop(0)
                logString = str(CurError.ErrorType) + "," + str(CurError.ErrorCode) + "," + str(CurError.OriginID) + "," + str(CurError.Timestamp) + "\n"

                # values for updating error counters 
                found = False
                listIndex = 0
                if CurError.OriginID-1 in range(GlobalVals.N_REAL_BALLOON):
                    saveIndex = CurError.OriginID-1
                else:
                    saveIndex = GlobalVals.N_REAL_BALLOON

                errorLen = len(errorTypes[saveIndex])
                
                # find the current error in the counters 
                if errorLen > 0:
                    for x in range(errorLen):
                        if errorTypes[saveIndex][x] == CurError.ErrorType:
                            listIndex = x
                            found = True
                            break
                
                # increment the counter 
                if not found:
                    errorTypes[saveIndex].append(CurError.ErrorType)
                    if CurError.ErrorType == 3:
                        errorTypeCount[saveIndex].append(CurError.ErrorCode)
                    else:
                        errorTypeCount[saveIndex].append(1)

                else:
                    if CurError.ErrorType == 3:
                        errorTypeCount[saveIndex][listIndex] = errorTypeCount[saveIndex][listIndex] + CurError.ErrorCode
                    else:
                        errorTypeCount[saveIndex][listIndex] = errorTypeCount[saveIndex][listIndex] + 1
                # if CurError.OriginID == 2:
                #     print('here')
                # write the string to file  
                try:
                    fileObj = open(GlobalVals.ERROR_LOG_FILE, "a")
                    fileObj.write(logString)
                    fileObj.close()
                except Exception as e:
                    print("Exception: " + str(e.__class__))
                    print("Error using error log file, ending error thread")
                    with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
                        GlobalVals.BREAK_ERROR_THREAD = True

        # Log the packet stats 
        curTime = time.time()
        if curTime >= statTime:

            # set the next time 
            statTime = curTime + GlobalVals.PACKET_STATS_INTERVAL
            
            # get the current packet count 
            PacketCount = 0
            with GlobalVals.PACKET_COUNT_MUTEX:
                PacketCount = GlobalVals.PACKET_COUNT
                GlobalVals.PACKET_COUNT = [0]*(GlobalVals.N_REAL_BALLOON+1)
            
            curTime_int = int(curTime)

            # open file 
            try:
                fileObj = open(GlobalVals.PACKET_STATS_FILE, "a")
            except Exception as e:
                print("Error Reporter: Error opening packet stats log. Now breaking thread.")
                print("Error Reporter:Exception: " + str(e.__class__))
                with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
                    GlobalVals.BREAK_ERROR_THREAD = True
                break
            
            # make string to be written to file 
            mainStr = str(curTime_int)
            packetStatsPercent = [0]* (GlobalVals.N_REAL_BALLOON+1)

            for i in range(GlobalVals.N_REAL_BALLOON+1):
                for x in range(len(errorTypes[i])):
                    tempStr = "," + str(i+1) + "," + str(errorTypes[i][x]) + "," + str(errorTypeCount[i][x]) + "/" + str(PacketCount[i]) 
                    mainStr = mainStr + tempStr
                    packetStatsPercent[i] = packetStatsPercent[i] + errorTypeCount[i][x]/PacketCount[i]
            mainStr = mainStr + "\n"
            
            with GlobalVals.PACKET_STATS_LOG_MUTEX:
                GlobalVals.PACKET_STATS_LOG = packetStatsPercent

            # write to file 
            try:
                fileObj.write(mainStr)
            except Exception as e:
                print("Error Reporter: Error writting to packet stats log. Now breaking thread.")
                print("Error Reporter:Exception: " + str(e.__class__))
                with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
                    GlobalVals.BREAK_ERROR_THREAD = True
                break
            finally:
                fileObj.close()
            
            # # clear error counts 
            # errorTypes.clear()
            # errorTypeCount.clear()
            errorTypes = [[] for _ in range(GlobalVals.N_REAL_BALLOON+1)]
            errorTypeCount = [[] for _ in range(GlobalVals.N_REAL_BALLOON+1)]

        # check if the thread needs to break  
        with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
            if GlobalVals.BREAK_ERROR_THREAD:
                break
