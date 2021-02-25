# import libraries 
import time
from threading import Thread, Lock
from dataclasses import dataclass

# import files 
import GlobalVals
import CustMes
import NetworkManager
import ErrorReporter

#=====================================================
# Main Code 
#=====================================================
if __name__ == '__main__':

    GlobalVals.PORT = "COM15"
    GlobalVals.SYSTEM_ID = 1
    TargetID1 = 2
    TargetID2 = 3

    # Start serial thread 
    NetworkThread = Thread(target=NetworkManager.SerialManagerThread,args=())
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.start()

    try:
        
        while True:
            
            # ping target 
            ping1 = NetworkManager.PingTarget(TargetID1)
            
            # if there is an error 
            if ping1 < 0:
                print("Ping Error: " + str(ping1))
                break

            # if no error print ping 
            else:
                print("Ping of System " + str(TargetID1) + " = " + str(ping1))
            
            # ping target 
            ping2 = NetworkManager.PingTarget(TargetID2)
            
            # if there is an error 
            if ping2 < 0:
                print("Ping Error: " + str(ping2))
                break

            # if no error print ping 
            else:
                print("Ping of System " + str(TargetID2) + " = " + str(ping2))
            
            time.sleep(1)

    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            GlobalVals.BREAK_NETWORK_THREAD = True

    # Safely end the serial thread
    if NetworkThread.is_alive():
        with GlobalVals.BREAK_NETWORK_THREAD_MUTEX:
            GlobalVals.BREAK_NETWORK_THREAD = True
        NetworkThread.join()
    
    # Safely end the error thread 
    if ErrorThread.is_alive():
        with GlobalVals.BREAK_ERROR_THREAD_MUTEX:
            GlobalVals.BREAK_ERROR_THREAD = True
        ErrorThread.join()