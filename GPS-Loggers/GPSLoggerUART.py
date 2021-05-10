import serial
from threading import Thread, Lock 
import time
import datetime
import pynmea2
import calendar
import GlobalVals
import socket
import struct
import subprocess

import sys, os
sys.path.insert(1,'../utils')
from utils import get_port
from common import *
from common_class import *
from commonGPS import *
#=====================================================
# Serial Thread
#=====================================================

def GPS_QuectelThread():
    
    # Initialise variables 
    readBytes = bytearray()
    connected = True
    bufferRead = 1
    synced = False

    # Connect to the serial port 
    try:
        serial_port = serial.Serial(
            port=GlobalVals.GPS_QUECTEL_UART_PORT,
            baudrate=GlobalVals.GPS_QUECTEL_UART_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=GlobalVals.GPS_UART_TIMEOUT,
        )
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
    except Exception as e:
        print("ERROR: unable to initiate serial port.")
        print("PORT = ", GlobalVals.GPS_QUECTEL_UART_PORT)
        print("BAUDRATE = ", GlobalVals.GPS_QUECTEL_UART_BAUDRATE)
        print("TIMEOUT = ", GlobalVals.GPS_QUECTEL_UART_TIMEOUT)
        print("Exception: " + str(e.__class__))
        connected = False
    
    # Wait a second to let the port initialize
    time.sleep(1)

    # begin the loop used for reading the serial port 
    while connected and not GlobalVals.EndGPS_QuectelSerial:
        
        # reset values when not synced
        if not synced:
            readBytes.clear()
            bufferRead = 1

        # read some bytes out of the buffer 
        try:
            comOut = serial_port.read(size=bufferRead)
            # print(comOut)
        except (OSError, serial.SerialException) as e:
            print("ERROR: Failed to read com port.")
            print("Exception: " + str(e.__class__))
            connected = False
            continue
        
        # if buffer is empty read again (likely timeout)
        if len(comOut) == 0:
            continue 
            
        # find the start of the messages ($ = 0x24) to sync the input buffer with the start of the message
        if not synced:
            if comOut[0] == 0x24:
                synced = True
                readBytes.append(comOut[0])
                bufferRead = 5  # message type is 5 bytes long
                continue
            else:
                synced = False
                continue 

        # Read the message type (GPGGA is what we want)
        if synced and bufferRead == 5:
            
            # check the message type is correct 
            messageType = "GPGGA"
            messageType_bytes = bytearray(messageType, 'utf-8')
            correctMessage = True
            for x in range(bufferRead):
                if comOut[x] != messageType_bytes[x]:
                    correctMessage = False
                    break
            
            # if it is not correct loop back 
            if not correctMessage:
                synced = False
                continue
            
            # add message type to readBytes 
            for x in comOut:
                readBytes.append(x)

            bufferRead = 1
            continue 
            
        # read the rest of the message (\r\n = 0x0D 0x0A is used to mark the end of a line)
        if bufferRead == 1 and synced:
            
            # if the byte isn't a CR then continue to add the bytes to readBytes
            if comOut[0] != 0x0D:
                readBytes.append(comOut[0])
                continue

            # Decode message (if the serial data has garbage, it will be discarded here)
            try:
                GGAstring = readBytes.decode()
            except Exception as e:
                print("Exception: " + str(e.__class__))
                print("Decoder broke, discarding packet and trying again.")
                synced = False
                continue
            
            # parse the GGA message 
            GGamessage = pynmea2.parse(GGAstring)
            # print("QuecTel: ", GGamessage)
            # add message to buffer
            with GlobalVals.GGA_QuectelBufferMutex:
                while len(GlobalVals.GGA_QuectelBuffer) > GlobalVals.GPS_UART_BUFFER_SIZE:
                    GlobalVals.GGA_QuectelBuffer.pop(0)
                GlobalVals.GGA_QuectelBuffer.append(GGamessage)
                

            # Update flag 
            with GlobalVals.NEWGPS_QuectelDataMutex:
                GlobalVals.NEWGPS_QuectelData = True
            
            synced = False
    
    # close thread
    if connected:
        serial_port.close()
    
    with GlobalVals.EndGPS_QuectelSerialMutex:
        GlobalVals.EndGPS_QuectelSerial = True

    return


#=====================================================
# Serial Thread
#=====================================================

def GPSSerialThread():
    
    # Initialise variables 
    readBytes = bytearray()
    connected = True
    bufferRead = 1
    synced = False

    # Connect to the serial port 
    try:
        serial_port = serial.Serial(
            port=GlobalVals.GPS_UART_PORT,
            baudrate=GlobalVals.GPS_UART_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=GlobalVals.GPS_UART_TIMEOUT,
        )
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
    except Exception as e:
        print("ERROR: unable to initiate serial port.")
        print("PORT = ", GlobalVals.GPS_UART_PORT)
        print("BAUDRATE = ", GlobalVals.GPS_UART_BAUDRATE)
        print("TIMEOUT = ", GlobalVals.GPS_UART_TIMEOUT)
        print("Exception: " + str(e.__class__))
        connected = False
    
    # Wait a second to let the port initialize
    time.sleep(1)

    # begin the loop used for reading the serial port 
    while connected and not GlobalVals.EndGPSSerial:
        
        # reset values when not synced
        if not synced:
            readBytes.clear()
            bufferRead = 1

        # read some bytes out of the buffer 
        try:
            comOut = serial_port.read(size=bufferRead)
            # print(comOut)
        except (OSError, serial.SerialException) as e:
            print("ERROR: Failed to read com port.")
            print("Exception: " + str(e.__class__))
            connected = False
            continue
        
        # if buffer is empty read again (likely timeout)
        if len(comOut) == 0:
            continue 
            
        # find the start of the messages ($ = 0x24) to sync the input buffer with the start of the message
        if not synced:
            if comOut[0] == 0x24:
                synced = True
                readBytes.append(comOut[0])
                bufferRead = 5  # message type is 5 bytes long
                continue
            else:
                synced = False
                continue 

        # Read the message type (GPGGA is what we want)
        if synced and bufferRead == 5:
            
            # check the message type is correct 
            messageType = "GNGGA"
            messageType_bytes = bytearray(messageType, 'utf-8')
            correctMessage = True
            for x in range(bufferRead):
                if comOut[x] != messageType_bytes[x]:
                    correctMessage = False
                    break
            
            # if it is not correct loop back 
            if not correctMessage:
                synced = False
                continue
            
            # add message type to readBytes 
            for x in comOut:
                readBytes.append(x)

            bufferRead = 1
            continue 
            
        # read the rest of the message (\r\n = 0x0D 0x0A is used to mark the end of a line)
        if bufferRead == 1 and synced:
            
            # if the byte isn't a CR then continue to add the bytes to readBytes
            if comOut[0] != 0x0D:
                readBytes.append(comOut[0])
                continue

            # Decode message (if the serial data has garbage, it will be discarded here)
            try:
                GGAstring = readBytes.decode()
            except Exception as e:
                print("Exception: " + str(e.__class__))
                print("Decoder broke, discarding packet and trying again.")
                synced = False
                continue
            
            # parse the GGA message 
            GGamessage = pynmea2.parse(GGAstring)
            # print(GGamessage)

            # add message to buffer
            with GlobalVals.GGABufferMutex:
                while len(GlobalVals.GGAbuffer) > GlobalVals.GPS_UART_BUFFER_SIZE:
                    GlobalVals.GGAbuffer.pop(0)
                GlobalVals.GGAbuffer.append(GGamessage)
            
            # Update flag 
            with GlobalVals.NewGPSData_Mutex:
                GlobalVals.NewGPSData = True
            
            synced = False
    
    # close thread
    if connected:
        serial_port.close()
    
    with GlobalVals.EndGPSSerial_Mutex:
        GlobalVals.EndGPSSerial = True

    return

#=====================================================
# Socket Thread
#=====================================================
def LoggerSocket():

    # start socket 
    Logger_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Logger_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    Logger_Socket.bind((GlobalVals.HOST, GlobalVals.GPS_LOGGER_SOCKET))
    Logger_Socket.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)



    # Wait for connection on logger socket (radio network scripts) 
    try:
        Logger_Socket.listen(1) 
        Logger_Connection, addr = Logger_Socket.accept()  
        Logger_Connection.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT) 
        print("Logger Connected to ", addr)                                            
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.EndGPSSocket_Mutex:
            GlobalVals.EndGPSSocket = True
        return 
    
    # local flags
    newData = False
    breakThread = False

    # forever while loop (until it is flagged to stop and break the thread)
    while not breakThread:

        # Check if the socket needs to end 
        with GlobalVals.EndGPSSocket_Mutex:
            if GlobalVals.EndGPSSocket:
                breakThread = True
                continue

        # check if there is new data
        with GlobalVals.NewGPSSocketData_Mutex:
            newData = GlobalVals.NewGPSSocketData
            GlobalVals.NewGPSSocketData = False

        # if there is new data 
        if newData:
            
            SocketPayload = bytearray()

            # go through all the new GPS data and append to one byte array
            with GlobalVals.GPSValuesMutex:
                while len(GlobalVals.GPSAltitude) > 0:
                    
                    # get the GPS values 
                    Longitude = GlobalVals.GPSLongitude.pop(0)
                    Latitude = GlobalVals.GPSLatitude.pop(0)
                    Altitude = GlobalVals.GPSAltitude.pop(0)
                    GPSTime = GlobalVals.GPSTimestamp.pop(0)

                    Longitude_ints = struct.pack('!d',Longitude)
                    Latitude_ints = struct.pack('!d',Latitude)
                    Altitude_ints = struct.pack('!d',Altitude)
                    GPSTime_ints = struct.pack('!d',GPSTime)

                    # append the sync header to the byte array (0xAA 0x55)
                    SocketPayload.append(0xAA)
                    SocketPayload.append(0x55)

                    # append the values to the byte array for the payload 
                    for x in Longitude_ints:
                        SocketPayload.append(x)
                    
                    for x in Latitude_ints:
                        SocketPayload.append(x)

                    for x in Altitude_ints:
                        SocketPayload.append(x)

                    for x in GPSTime_ints:
                        SocketPayload.append(x)
            
            # send data over socket
            SocketPayload = bytes(SocketPayload)
            try:
                Logger_Connection.sendall(SocketPayload)
            except Exception as e:
                print("Exception: " + str(e.__class__))
                print("Error in the logger socket. Now closing thread.")
                breakThread = True
                break

        # if there is no new data sleep for 0.5 seconds      
        else:
            time.sleep(0.5)
    
    # if the thread is broken set the global flag 
    if breakThread:
        with GlobalVals.EndGPSSocket_Mutex:
            GlobalVals.EndGPSSocket = True
    
    # close connection before ending thread 
    Logger_Connection.close()
        
#=====================================================
# Main Function of script 
#=====================================================

# the main thread loop function 
def main():
    
    # setup time values 
    curTime = time.time()
    callTime = curTime + GlobalVals.PREDICTION_INTERVAL
    firstRunUblox = True
    statusUblox = True

    # loop only while the other threads are running 
    while not GlobalVals.EndGPSSerial and not GlobalVals.EndGPSSocket:

        # check to see if there is new data 
        dataUbloxReady = False
        dataQuectelRead = False

        with GlobalVals.NewGPSData_Mutex:
            if GlobalVals.NewGPSData:
                dataUbloxReady = True 
                GlobalVals.NewGPSData = False

        with GlobalVals.NEWGPS_QuectelDataMutex:
            if GlobalVals.NEWGPS_QuectelData:
                dataQuectelRead = True
                GlobalVals.NEWGPS_QuectelData = False
                        
        
        # if there is no new data sleep 
        if not dataUbloxReady and not dataQuectelRead:
            time.sleep(0.1)
            continue 

        with GlobalVals.GGABufferMutex:
            
            # loop through each value in buffer
            loopLength = len(GlobalVals.GGAbuffer)
            if loopLength > 0:
                statusUblox = True
                while loopLength > 0:
                    
                    # Get GPS data from the value in buffer
                    GGAdata = GlobalVals.GGAbuffer.pop(0)
                    GPS_Data = GGA_Convert(GGAdata)
                    obtainedUbloxCheck = checkGPS(GPS_Data)

                    with GlobalVals.GPSValuesMutex:
        
                        if obtainedUbloxCheck or firstRunUblox:
                            firstRunUblox = False
                        # if obtainedUbloxCheck:
                            print('Using Ublox GPS')
                            updateGlobalGPS_Data(GPS_Data)
                        else:
                            with GlobalVals.GGA_QuectelBufferMutex:
                                # loop through each value in buffer
                                loopLengthQuectel = len(GlobalVals.GGA_QuectelBuffer)
                                while loopLengthQuectel > 0:                                    
                                    # Get GPS data from the value in buffer
                                    GGAdataQuectel = GlobalVals.GGA_QuectelBuffer.pop(0)
                                    
                                    GPS_Data = GGA_Convert(GGAdataQuectel)
                                    obtainedQuectelCheck = checkGPS(GPS_Data)
                                        
                                    if obtainedQuectelCheck:
                                        if GPS_Data.epoch > GlobalVals.GPSTimestamp[-1]:
                                            print('Using QuecTel GPS')
                                            updateGlobalGPS_Data(GPS_Data)

                                    loopLengthQuectel = loopLengthQuectel -1

                    # set the flag for the socket code 
                    with GlobalVals.NewGPSSocketData_Mutex:
                        GlobalVals.NewGPSSocketData = True
                            
                    # Log the GPS data
                    logData(GPS_Data)                                                                             
                    loopLength = loopLength - 1

            if time.time() - GPS_Data.epoch > GlobalVals.UBLOX_SIGNAL_LOSS_TIME:
                statusUblox = False

            if not statusUblox:
                with GlobalVals.GPSValuesMutex:
                    with GlobalVals.GGA_QuectelBufferMutex:     
                        # loop through each value in buffer
                        loopLengthQuectel = len(GlobalVals.GGA_QuectelBuffer)
                        while loopLengthQuectel > 0:
                            
                            # Get GPS data from the value in buffer
                            GGAdataQuectel = GlobalVals.GGA_QuectelBuffer.pop(0)
                            
                            GPS_Data = GGA_Convert(GGAdataQuectel)
                            obtainedQuectelCheck = checkGPS(GPS_Data)
                                
                            if obtainedQuectelCheck:
                                if GPS_Data.epoch > GlobalVals.GPSTimestamp[-1]:
                                    print('Using QuecTel GPS 2')
                                    updateGlobalGPS_Data(GPS_Data)
                            
                            with GlobalVals.NewGPSSocketData_Mutex:
                                GlobalVals.NewGPSSocketData = True
                    
                            # Log the GPS data
                            logData(GPS_Data)

                            loopLengthQuectel = loopLengthQuectel -1

                            # set the flag for the socket code 
                            

            # check if it is call time yet
            curTime = time.time()
            if curTime >= callTime:
                callTime = curTime + GlobalVals.PREDICTION_INTERVAL

                # if real GPS data is being used 
                if not GlobalVals.FAKE_GPS_FLAG:
                    
                    # check to see if useful data has been loaded 
                    if GPS_Data.lon != 0.0 and GPS_Data.lat != 0.0 and GPS_Data.alt != 0.0:
                        
                        # lat on alt ascent epoch
                        # /home/pi/LuxCode/Prediction_Autologger/build/BallARENA-dev -36.71131 142.19981 420 6.9 1612614684 .
                        print("Calling Path Prediction.") 
                        argStr = str(GPS_Data.lat) + " " + str(GPS_Data.lon) + " " + str(GPS_Data.alt) + " " + str(GlobalVals.GPSAscentRate) + " " + str(int(GPS_Data.epoch)) 
                        commandStr = "~/HAB/Prediction_Autologger/build/BallARENA-dev " + argStr
                        
                        try:
                            subprocess.Popen(commandStr, shell=True)
                        except Exception as e:
                            print("Exception: " + str(e.__class__))
                            print(e)
                            print("Error calling path prediction script. Will continue and call again later.")
                
                else:
                    # fake gps over launch site
                    print("Calling Path Prediction.") 

                    try:
                        subprocess.Popen('~/HAB/Prediction_Autologger/build/BallARENA-dev -36.71131 142.19981 4200 6.9 1612614684', shell=True)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print(e)
                        print("Error calling path prediction script. Will continue and call again later.")




# the main file code starts here 
if __name__ == '__main__':
    numArgs = len(sys.argv)

    # set Port
    if numArgs >= 2:
        GlobalVals.GPS_UART_PORT = sys.argv[1]
    else:
        GlobalVals.GPS_UART_PORT=get_port('GPS')
    
    print('PORT: '+ GlobalVals.GPS_UART_PORT)

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-GPSLogger.csv"
    GlobalVals.GPS_LOGGER_FILE = file_name

    logString = "epoch, lon, lat, alt, ascent_rate \n"

    try:
        fileObj = open(GlobalVals.GPS_LOGGER_FILE, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error using error log file, ending error thread")

    # start the serial thread 
    GPS_UbloxThread = Thread(target=GPSSerialThread, args=())
    GPS_UbloxThread.start()
    # GPS_QuectelThread()
    
    # start the serial thread 
    GPS_QuecTelThread = Thread(target=GPS_QuectelThread, args=())
    GPS_QuecTelThread.start()

    # # start the socket logger thread 
    # SocketThread = Thread(target=LoggerSocket, args=())
    # SocketThread.start()
    
    # start the main loop 
    try:
        main()
    except KeyboardInterrupt:
        print('\nProgram ended.')
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print(e)

    # safely end the GPS thread 
    if GPS_UbloxThread.is_alive():
        with GlobalVals.EndGPSSerial_Mutex:
            GlobalVals.EndGPSSerial = True
        GPS_UbloxThread.join()
    
    # safely end the GPS thread 
    if GPS_QuecTelThread.is_alive():
        with GlobalVals.EndGPS_QuectelSerialMutex:
            GlobalVals.EndGPS_QuectelSerial = True
        GPS_QuecTelThread.join()

    # # safely end the socket thread 
    # if SocketThread.is_alive():
    #     with GlobalVals.EndGPSSocket_Mutex:
    #         GlobalVals.EndGPSSocket = True
    #     GPSThread.join()
        

