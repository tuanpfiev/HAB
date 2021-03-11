import serial
import time
import sys, os
import GlobalVals

sys.path.insert(1,'../utils')
from utils import get_port


def main(StartState):
    
    connected = False 
    
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
        connected = True
    except Exception as e:
        print("LoRa Radio: Unable to initiate serial port. Now breaking thread.")
        print("LoRa Radio: Exception: " + str(e.__class__))
        connected = False
    
    time.sleep(1)

    #intiallise Variables 
    waiting = StartState
    syncA = False
    syncB = False
    syncC = False
    handshake = False
    silent = False
    curTime = time.time()
    waitLimit = curTime + GlobalVals.WAITING_TIMEOUT

    # Handshake loop
    while connected:

        # When a handshake is recieved record the rssi
        if handshake:
            
            handshakeTime = int(time.time())

            # send RSSI command  
            try:
                serial_port.write(GlobalVals.RSSI_COMMAND)
            except Exception as e:
                print("LoRa Radio: Unable to write to serial port. Now breaking thread.")
                print("LoRa Radio: Exception: " + str(e.__class__))
                connected = False
                break

            # read line from serial port (the response has line return at the end) 
            try:
                dataOut = serial_port.readline()
            except Exception as e:
                print("LoRa Radio: Unable to read serial port. Now breaking thread.")
                print("LoRa Radio: Exception: " + str(e.__class__))
                connected = False
                break
            
            # print response 
            print(dataOut)

            # check if the dataout varibel is big enough to check 
            if (len(dataOut) < 2):
                print ("LoRa Radio: Recieved data is too small. Discarding current line.")

            # check if the data recieved is aligned
            if (len(dataOut) >= 2): 
                if dataOut[0] != 0xAF or dataOut[1] != 0xAF:
                    print ("LoRa Radio: Packet is not aligned. Discarding current line.")
            
                # otherwise 
                elif not silent:
                    
                    # get rssi
                    rssi = int(dataOut[8]) - 164
                    print("RSSI = " + str(rssi))

                    # format log string 
                    logString = str(handshakeTime) + "," + str(rssi) + "\n"

                    # write log string to file  
                    try:
                        fileObj = open(GlobalVals.RSSI_LOG_FILE, "a")
                        fileObj.write(logString)
                        fileObj.close()
                    except Exception as e:
                        print("LoRa Radio: Error writting to file. Breaking thread.")
                        print("LoRa Radio: Exception: " + str(e.__class__))
                        break
                
                elif silent:
                    silent = False

            handshake = False

        # if not waiting (Therefore sending the handshake)
        if not waiting: 

            # send handshake 
            try:
                serial_port.write(GlobalVals.HANDSHAKE_BYTES)
            except Exception as e:
                print("LoRa Radio: Unable to write to serial port. Now breaking thread.")
                print("LoRa Radio: Exception: " + str(e.__class__))
                break
            
            print("Sent Handshake.")

            # set waiting flag and witing time 
            waiting = True
            curTime = time.time()
            waitLimit = curTime + GlobalVals.WAITING_TIMEOUT
            time.sleep(0.5)
            continue
        
        # if it is waiting 
        else:

            # read incoming data 
            try:
                dataOut = serial_port.read(size=1)
            except Exception as e:
                print("LoRa Radio: Unable to read serial port. Now breaking thread.")
                print("LoRa Radio: Exception: " + str(e.__class__))
                connected = False
                break
            
            # if there is something in the output 
            if len(dataOut) != 0:

                print(dataOut)
                # find handshake 
                if not handshake:
                    if dataOut[0] == GlobalVals.HANDSHAKE_BYTES[0] and not syncA:
                        syncA = True
                        continue
                    elif dataOut[0] == GlobalVals.HANDSHAKE_BYTES[1] and not syncB:
                        syncB = True
                        continue
                    elif dataOut[0] == GlobalVals.HANDSHAKE_BYTES[2] and not syncC:
                        syncC = True
                    else:
                        syncA = False
                        syncB = False
                        syncC = False
                    
                    # if all parts have been found set hand shake to true
                    if syncA and syncB and syncC:
                        handshake = True 
                        syncA = False
                        syncB = False
                        syncC = False
                        waiting = False
                        print ("Recieved Handshake.")

            
            # if there is nothing in the output (likely a timeout)
            else:
                
                # if the script has waited more then the wiat time for a response send a new handshake 
                curTime = time.time()
                if curTime >= waitLimit:
                    handshake = True 
                    syncA = False
                    syncB = False
                    syncC = False
                    waiting = False
                    silent = True

            

if __name__ == '__main__':

    # get arguments for running the script
    starter = False
    numArgs = len(sys.argv)
    
    # check if the number of args are correct 
    # if numArgs != 3:
    #     print("Incorrect number of Args.")
    #     print(numArgs)
    #     sys.exit() 
    
    # check if this script will start the handshake or not
    if sys.argv[1] == 'start':
        starter = True
    elif sys.argv[1] == 'wait':
        starter = False
    else:
        print("Incorrect first arg.")
        sys.exit() 
    
    # use the third argument as the com port 

    if numArgs == 3:
        GlobalVals.PORT = sys.argv[2]
    else:
        GlobalVals.PORT = get_port('Lora')
        print('PORT: '+ GlobalVals.PORT)
    
    # create log file string 
    data_log_folder_name = sys.argv[1]

    try:
        os.makedirs(data_log_folder_name)
    except FileExistsError:
        pass

    file_name = data_log_folder_name+"/"+time.strftime("%Y%m%d-%H%M%S")+"-RSSILog.txt"
    GlobalVals.RSSI_LOG_FILE = file_name
    print(GlobalVals.RSSI_LOG_FILE)

    # run the main function until something goes wrong 
    try:
        main(not starter)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")
        


