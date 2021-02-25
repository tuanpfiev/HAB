# import libraries 
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

    GlobalVals.PORT = "COM16"
    GlobalVals.SYSTEM_ID = 2
    GlobalVals.ERROR_LOG_FILE = "ErrorLog2.txt"

    # Start network manager 
    NetworkThread = Thread(target=NetworkManager.SerialManagerThread,args=())
    NetworkThread.start()

    # Start error logger 
    ErrorThread = Thread(target=ErrorReporter.ErrorLogger, args=())
    ErrorThread.start()
    
    # loop forever 
    try:
        while True:
            pass    
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
