
import serial
import time
import subprocess

import GlobalVals


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

            # check if the data recieved is aligned 
            if dataOut[0] != 0xAF or dataOut[1] != 0xAF:
                print ("LoRa Radio: Packet is not aligned. Discarding current line.")
            
            # otherwise 
            else:
                
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
