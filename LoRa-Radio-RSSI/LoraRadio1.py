import GlobalVals
import os, sys
import time

sys.path.insert(1,'../utils')
from utils import get_port
from func_main import main


if __name__ == '__main__':

    # set the serial port 
    GlobalVals.PORT = get_port('Lora')
    print('PORT: '+ GlobalVals.PORT)
    # Log file
    data_log_folder_name = "datalog1"

    try:
        os.makedirs(data_log_folder_name)
    except FileExistsError:
        pass

    file_name = data_log_folder_name+"/"+time.strftime("%Y%m%d-%H%M%S")+"-RSSILog.txt"
    GlobalVals.RSSI_LOG_FILE = file_name

    # run the main function until something goes wrong 
    try:
        main(False)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")