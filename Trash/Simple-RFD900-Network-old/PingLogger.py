# import libraries 
import time

# import files 
import GlobalVals
import NetworkManager

#=====================================================
# Ping Logger Thread Function
#=====================================================
def PingLoggerThread():
 
    pingTime = time.time() + GlobalVals.PING_INTERVAL
    timeToPing = False

    while True:

        with GlobalVals.BREAK_PING_THREAD_MUTEX:
            if GlobalVals.BREAK_PING_THREAD:
                break

        currentTime = time.time()

        if currentTime >= pingTime:
             timeToPing = True 
        else:
            time.sleep(0.2)
            continue

        if timeToPing:
            
            # change values
            timeToPing = False
            pingTime = time.time() + GlobalVals.PING_INTERVAL
            pingStrings = []

            # get list of system IDs
            systemList = NetworkManager.RequestSystemsList()
            if len(systemList) == 0:
                continue
            
            # get ping for each system 
            for x in systemList:
                ping = NetworkManager.PingTarget(x)
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
                print("Ping Logger: Error opening file. Now breaking thread.")
                print("Ping Logger: Exception: " + str(e.__class__))
                with GlobalVals.BREAK_PING_THREAD_MUTEX:
                    GlobalVals.BREAK_PING_THREAD = True
                break
            
            # Write to log file 
            try: 
                for x in pingStrings:
                    fileObj.write(x)
            except Exception as e:
                print("Ping Logger: Error writting to file. Now breaking thread.")
                print("Ping Logger: Exception: " + str(e.__class__))
                with GlobalVals.BREAK_PING_THREAD_MUTEX:
                    GlobalVals.BREAK_PING_THREAD = True
                break
            finally:
                fileObj.close()
            

            

