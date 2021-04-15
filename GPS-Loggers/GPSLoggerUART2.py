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
    print('here')
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
                bufferRead = 14  # message type is 5 bytes long
                continue
            else:
                synced = False
                continue 

        # Read the message type (GPGGA is what we want)
        if synced and bufferRead == 14:
            print(comOut)
    
            # check the message type is correct 
            messageType = "GP"
            messageType_bytes = bytearray(messageType, 'utf-8')
            correctMessage = True
            for x in range(2):
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

    # loop only while the other threads are running 
    while not GlobalVals.EndGPSSerial and not GlobalVals.EndGPSSocket:

        # check to see if there is new data 
        DataReady = False
        with GlobalVals.NewGPSData_Mutex:
            if GlobalVals.NewGPSData:
                DataReady = True 
                GlobalVals.NewGPSData = False
        
        # if there is no new data sleep 
        if not DataReady:
            time.sleep(0.1)
            continue 

        with GlobalVals.GGABufferMutex:
            
            # loop through each value in buffer
            loopLength = len(GlobalVals.GGAbuffer)
            while loopLength > 0:
                
                # Get GPS data from the value in buffer
                GGAdata = GlobalVals.GGAbuffer.pop(0)
                lon = GGAdata.longitude
                lat = GGAdata.latitude
                alt = 0.0
                GPStime_hour = 0
                GPStime_min = 0
                GPStime_sec = 0
                GPSepoch = 0.0

                # if there is no data in the timestamp feild of the message then the timestamp has no type. 
                if GGAdata.data[0] != '':
                    
                    # set the alt
                    if GGAdata.altitude != None:
                        alt = GGAdata.altitude

                    # get the hours, minutes, and seconds from GPS
                    GPStime_hour = GGAdata.timestamp.hour
                    GPStime_min = GGAdata.timestamp.minute
                    GPStime_sec = GGAdata.timestamp.second

                    # from the GPS calculate the seconds in the day 
                    DaySecs = (GPStime_hour * 3600) + (GPStime_min * 60) + GPStime_sec
                    
                    # get epoch time for UTC at the start of the day 
                    UTCtime = datetime.datetime.utcnow()
                    UTCtimeEpoch = int(calendar.timegm(UTCtime.timetuple()))
                    UTCDaySecs = (UTCtime.hour * 3600) + (UTCtime.minute * 60) + UTCtime.second
                    UTCDayStart = UTCtimeEpoch - UTCDaySecs
                    
                    # get GPS epoch
                    GPSepoch = UTCDayStart + DaySecs
                    
                    # if the time is conversion happens over midnight (GPS = 23:59:59, time of conversion = 00:00:00)
                    # then ajust the start of the day 
                    UTCtimeEpoch = UTCtimeEpoch + 60    # seems that the time gps doesn't line up exactly so a 1 minute error range helps with this 
                    if GPSepoch > UTCtimeEpoch:
                        UTCDayStart = UTCDayStart - 86400
                        GPSepoch = UTCDayStart + DaySecs
                    
                    GPSepoch = float(GPSepoch)
                
                # Load data into the buffer
                with GlobalVals.GPSValuesMutex:
                    GlobalVals.GPSLongitude.append(lon)
                    GlobalVals.GPSLatitude.append(lat)
                    GlobalVals.GPSAltitude.append(alt)
                    GlobalVals.GPSTimestamp.append(GPSepoch)
                    GlobalVals.GPSAscentRateVals.append(alt)

                    # calculate ascent rate if enough alt vals have been recorded 
                    NumAscentVals = len(GlobalVals.GPSAscentRateVals)
                    if NumAscentVals >= GlobalVals.GPS_LOGGER_ASCENT_RATE_LENGTH:
                        
                        ascentRate = 0 
                        for x in range(NumAscentVals):
                            if x == 0:
                                continue
                            
                            diff = GlobalVals.GPSAscentRateVals[x] - GlobalVals.GPSAscentRateVals[x-1]
                            ascentRate = ascentRate + diff

                        ascentRate = ascentRate / (NumAscentVals - 1)
                        
                        # store ascent rate and pop one value (running average)
                        GlobalVals.GPSAscentRate = ascentRate
                        GlobalVals.GPSAscentRateVals.pop(0)

                # set the flag for the socket code 
                with GlobalVals.NewGPSSocketData_Mutex:
                    GlobalVals.NewGPSSocketData = True
                
                # Log the GPS data
                logString = str(GPSepoch) + "," + str(lon) + "," + str(lat) + "," + str(alt) + "," + str(GlobalVals.GPSAscentRate) + "\n"
                timeLocal = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(GPSepoch))
                try:
                    fileObj = open(GlobalVals.GPS_LOGGER_FILE, "a")
                    fileObj.write(logString)
                    fileObj.close()
                except Exception as e:
                    print("Exception: " + str(e.__class__))
                    print("Error using error log file, ending error thread")
                    return

                # Debug Messages 
                print("lon: " + str(round(lon,4)) + ", lat: " + str(round(lat,4)) + ", alt: " + str(round(alt,2)) + ", Time: ", timeLocal)

                loopLength = loopLength - 1

                # check if it is call time yet
                curTime = time.time()
                if curTime >= callTime:
                    callTime = curTime + GlobalVals.PREDICTION_INTERVAL

                    # if real GPS data is being used 
                    if not GlobalVals.FAKE_GPS_FLAG:
                        
                        # check to see if useful data has been loaded 
                        if lon != 0.0 and lat != 0.0 and alt != 0.0:
                            
                            # lat on alt ascent epoch
                            # /home/pi/LuxCode/Prediction_Autologger/build/BallARENA-dev -36.71131 142.19981 420 6.9 1612614684 .
                            print("Calling Path Prediction.") 
                            argStr = str(lat) + " " + str(lon) + " " + str(alt) + " " + str(GlobalVals.GPSAscentRate) + " " + str(int(GPSepoch)) 
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

    # set Port
    GlobalVals.GPS_UART_PORT=get_port('GPS')
    print('PORT: '+ GlobalVals.GPS_UART_PORT)
    GlobalVals.GPS_UART_PORT="/dev/ttyUSB0"
    GlobalVals.GPS_UART_BAUDRATE = 38400

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
    # GPSThread = Thread(target=GPSSerialThread, args=())
    # GPSThread.start()

    GPSSerialThread()

    # # start the socket logger thread 
    # SocketThread = Thread(target=LoggerSocket, args=())
    # SocketThread.start()
    
    # start the main loop 
    # try:
    #     main()
    # except KeyboardInterrupt:
    #     print('\nProgram ended.')
    # except Exception as e:
    #     print("Exception: " + str(e.__class__))
    #     print(e)

    # # safely end the GPS thread 
    # if GPSThread.is_alive():
    #     with GlobalVals.EndGPSSerial_Mutex:
    #         GlobalVals.EndGPSSerial = True
    #     GPSThread.join()
    
    # safely end the socket thread 
    # if SocketThread.is_alive():
    #     with GlobalVals.EndGPSSocket_Mutex:
    #         GlobalVals.EndGPSSocket = True
    #     GPSThread.join()
        

