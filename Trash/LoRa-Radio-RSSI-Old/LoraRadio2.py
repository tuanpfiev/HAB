import GlobalVals
import os, sys
import time

sys.path.insert(1,'../utils')
from utils import get_port
from func_main import main

           

if __name__ == '__main__':

    # set the serial port 
    #GlobalVals.PORT = "COM25"
    #GlobalVals.RSSI_LOG_FILE = "RSSILog2.txt"
    GlobalVals.PORT = get_port('Lora')
    print('PORT: '+ GlobalVals.PORT)

    # Log file
    try:
        os.makedirs("datalog2")
    except FileExistsError:
        pass

    file_name = "datalog2/"+time.strftime("%Y%m%d-%H%M%S")+"-RSSILog.txt"
    GlobalVals.RSSI_LOG_FILE = file_name

    # run the main function until something goes wrong 
    try:
        main(True)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")
        


