import serial
import time
import sys, os
import GlobalVals
import numpy as np
import socket
from threading import Thread

sys.path.insert(1,'../utils')
from utils import get_port

class RSSI_Tracker:
    def __init__(self,x):
        self.R = 2
        self.H = np.array([1,0]).reshape(1,2)
        self.Q = 0.005*np.eye(2)
        self.P = np.eye(2)
        self.x = np.array([x,0]).reshape(2,1)
    def update(self,x,P,z,dt):
        xp = np.dot(np.array([[1,dt],[0,1]]),x)
        Pp = P + self.Q
        S = np.dot(np.dot(self.H,Pp),self.H.T) + self.R
        K = np.dot(np.dot(Pp,self.H.T),np.linalg.inv(S))
        info = z - np.dot(self.H,xp)
        corr = np.dot(K,info)
        x = xp + corr
        P = np.dot(np.eye(2) - np.dot(K,self.H),Pp)
        return x,P

global Tracker, x, P, prev_handshakeTime, filtered_RSSI
Tracker = RSSI_Tracker(0)
x = Tracker.x
P = Tracker.P
prev_handshakeTime = 0
filtered_RSSI = 0

def RSSI_to_distance(rssi):
    n = 1.7
    A = -60
    return 10**(-(rssi-A)/(n*10))


def main(StartState):
    global Tracker, x, P, prev_handshakeTime
    
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
        # DataReady = False
        with GlobalVals.NewRSSISocketData_Mutex:
            if GlobalVals.NewRSSISocketData:
                # DataReady = True
                GlobalVals.NewRSSISocketData = False

        # if not DataReady:
        #     time.sleep(0.1)
        #     continue


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

                    # Kalman filter for RSSI
                    
                    if prev_handshakeTime == 0:
                        dt = 1
                    else:
                        dt = handshakeTime - prev_handshakeTime
                        prev_handshakeTime = handshakeTime

                    x, P = Tracker.update(x, P, rssi, dt)
                    filtered_RSSI = x[0,0]
                    distance = RSSI_to_distance(filtered_RSSI)

                    with GlobalVals.RSSIValues_Mutex:
                        GlobalVals.RSSI_filtered.append(filtered_RSSI)
                        GlobalVals.distance.append(distance)
                        GlobalVals.RSSI_time.append(handshakeTime)

                    with GlobalVals.NewRSSISocketData_Mutex:
                        GlobalVals.NewRSSISocketData = True

                    # format log string 
                    logString = str(handshakeTime) + "," + str(rssi) + "," + str(filtered_RSSI) + "," + str(distance) + "\n"

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

def Thread_RSSI_publish(host,port):
     
    Logger_Socket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Logger_Socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    Logger_Socket2.bind((host, 5022))
    Logger_Socket2.settimeout(GlobalVals.SOCKET_TIMEOUT)

    try: 
        Logger_Socket2.listen(1)
        Logger_Connection2, addr2 = Logger_Socket2.accept()  
        Logger_Connection2.settimeout(GlobalVals.SOCKET_TIMEOUT) 
        print("Connected to: ",addr2)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
        return 
    
    
    Logger_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Logger_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    Logger_Socket.bind((host, port))
    Logger_Socket.settimeout(GlobalVals.SOCKET_TIMEOUT)

    try: 
        Logger_Socket.listen(1)
        Logger_Connection, addr = Logger_Socket.accept()  
        Logger_Connection.settimeout(GlobalVals.SOCKET_TIMEOUT) 
        print("Connected to: ",addr)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
        return
        

    breakThread = False
    newData = False

    while not breakThread:
        
        # Check if the socket ends
        with GlobalVals.EndRSSISocket_Mutex:
            if GlobalVals.EndRSSISocket:
                breakThread = True
                continue

        # Check if there is new data
        with GlobalVals.NewRSSISocketData_Mutex:
            newData = GlobalVals.NewRSSISocketData
            GlobalVals.NewRSSISocketData = False

        if newData:

            with GlobalVals.RSSIValues_Mutex:
                while len(GlobalVals.RSSI_filtered)>0:
                    RSSI_filtered = GlobalVals.RSSI_filtered.pop(0)
                    distance = GlobalVals.distance.pop(0)
                    RSSI_time = GlobalVals.RSSI_time.pop(0)

                    socketPayload = "{RSSI_filter: " + str(RSSI_filtered) + "; distance: " + str(distance) + "; time: " + str(RSSI_time) +";}"
                
                    socketPayload = socketPayload.encode("utf-8")
                
                    try:
                        Logger_Connection2.sendall(socketPayload)
                        Logger_Connection.sendall(socketPayload)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error in the logger socket. Now closing thread.")
                        breakThread = True
                        break
        else:
            time.sleep(0.01)

    # if the thread is broken set the global flag 
    if breakThread:
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
    
    # close connection before ending thread 
    Logger_Connection2.close()
    Logger_Connection.close()


if __name__ == '__main__':

    # get arguments for running the script
    starter = False
    numArgs = len(sys.argv)
    
    # check if the number of args are correct 
    # if numArgs != 3:filtered_RSSI
    #     print("Incorrect number of Args.")
    #     print(numArgs)
    #     sys.exit() 
    
    # check if this script will start the handshake or not
    # if sys.argv[1] == 'start':
    #     starter = True
    # elif sys.argv[1] == 'wait':
    #     starter = False
    # else:
    #     print("Incorrect first arg.")
    starter = False
        # sys.exit() 
    
    # use the third argument as the com port 

    if numArgs == 3:
        GlobalVals.PORT = sys.argv[2]
    else:
        GlobalVals.PORT = get_port('Lora')
        print('PORT: '+ GlobalVals.PORT)
    
    # create log file string 
    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-LoraRSSI.csv"
    GlobalVals.RSSI_LOG_FILE = file_name

    logString = "epoch, rssi, filtered_RSSI, distance \n"

    try:
        fileObj = open(GlobalVals.RSSI_LOG_FILE, "a")
        fileObj.write(logString)
        fileObj.close()
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error using error log file, ending error thread")


    RSSI_Thread = Thread(target = Thread_RSSI_publish, args = (GlobalVals.HOST,GlobalVals.PORT_RSSI))
    RSSI_Thread.start()
    # run the main function until something goes wrong 
    try:
        main(not starter)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")
        
    if RSSI_Thread.is_alive():
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
        RSSI_Thread.join()


