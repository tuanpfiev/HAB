import serial
import time
import sys, os
import GlobalVals
import numpy as np
import socket
from threading import Thread
import math
import copy
import numpy as np 



sys.path.insert(1,'../utils')
from utils import get_port
from navpy import lla2ned, ned2lla, lla2ecef
from common import *
from common_class import *

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

def checkGPS(gps):
    if gps.lat == 0.0 and gps.lon == 0.0 and gps.alt == 0.0:
        return False
    else:
        return True
        
def checkAllGPS(gps_list):
    for i in range(len(gps_list)):
        if not checkGPS(gps_list[i]):
            return False
    return True

def gps_update(new_data):
    i = sysID_to_index(new_data.sysID)
    GlobalVals.GPS_ALL[i-1] = new_data

def gps_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    
    while True:
        try:        
            s.connect((host,port))
            s.settimeout(GlobalVals.GPS_TIMEOUT)
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to GPS....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the GPS socket. This thread will now stop.")
                with GlobalVals.BREAK_GPS_THREAD_MUTEX:
                    GlobalVals.BREAK_GPS_THREAD = True
                return 
        break

    while True:
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.GPS_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the GPS receiver socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = []
        string_list = extract_str_btw_curly_brackets(data_str)

        if len(string_list) > 0:
            gps_list = []
            for string in string_list:
                received, gps_i = stringToGPS(string)
                if received:
                    gps_list.append(gps_i)
            
            idx = 0
            
            with GlobalVals.GPS_UPDATE_MUTEX:
                while idx < len(gps_list):
                    gps_update(gps_list[idx])
                    idx += 1

    s.close()

def RSSI_ToDistance(rssi,params):
    n = params[0][0]
    A = params[0][1]
    
    rssi.distance = 10**(-(rssi.rssi_filtered-A)/(n*10))

    return rssi

def distanceCalculation(gps1, gps2):
    p1 = lla2ecef(gps1.lat,gps1.lon,gps1.alt)
    p2 = lla2ecef(gps2.lat,gps2.lon,gps2.alt)
    return np.linalg.norm(p1-p2)


def RSSI_Calibration(rssi,gpsAll,sysID,targetBalloon):



    if not checkGPS(gpsAll[sysID-1]) or not checkGPS(gpsAll[targetBalloon-1]):
        if not GlobalVals.RSSI_CALIBRATION_FINISHED:
            print('GPS is not available!!!!!! Cannot calibrate RSSI!!!')
            time.sleep(0.5)
            return np.ones([1,2]), False, rssi
        else:
            print('GPS is not available!!!!!! Use the calibrated RSSI params to calculate the distance')

            rssi = RSSI_ToDistance(rssi,GlobalVals.RSSI_PARAMS)

            return GlobalVals.RSSI_PARAMS, True, rssi

    distance = distanceCalculation(gpsAll[sysID-1],gpsAll[targetBalloon-1])
    # print(sysID)
    # print(gpsAll[sysID-1].lon)
    # print(gpsAll[targetBalloon-1].lon)
    # print("--")

    GlobalVals.X = np.concatenate((GlobalVals.X,np.array([[rssi.rssi_filtered,1]])),axis=0)
    GlobalVals.Y = np.concatenate((GlobalVals.Y,np.array([[np.log10(distance)]])),axis=0)
    
    if len(GlobalVals.X)< GlobalVals.RSSI_CALIBRATION_SIZE:
        print("Calibrating RSSI sys ID: ",targetBalloon,"(",len(GlobalVals.X),"/",GlobalVals.RSSI_CALIBRATION_SIZE,")")
        return np.ones([1,2]), False, rssi

    if len(GlobalVals.X) > GlobalVals.RSSI_CALIBRATION_SIZE:
        GlobalVals.X = np.delete(GlobalVals.X,0,0)
        GlobalVals.Y = np.delete(GlobalVals.Y,0,0)

    # add 2 data point to avoid bifurcation
    # GlobalVals.X = np.concatenate((GlobalVals.X,np.array([[-1,1]])),axis=0)
    # GlobalVals.Y = np.concatenate((GlobalVals.Y,np.array([[np.log10(1)]])),axis=0)    # distance = 1m, rssi = -1

    # GlobalVals.X = np.concatenate((GlobalVals.X,np.array([[-130,1]])),axis=0)
    # GlobalVals.Y = np.concatenate((GlobalVals.Y,np.array([[np.log10(50000)]])),axis=0)    # distance = 20000m, rssi = -130

    w = np.linalg.multi_dot([np.linalg.inv(np.dot(GlobalVals.X.transpose(),GlobalVals.X)),GlobalVals.X.transpose(),GlobalVals.Y])
    n = -1/(10*w[0][0])
    A = 10*n*w[1][0]
    params = np.array([[n,A]])

    rssi = RSSI_ToDistance(rssi,params)
    if rssi.distance >10e10:
        xx = rssi.distance
    print("Calibrated RSSI sys ID: ",targetBalloon,", (GPS Distance, RSSI distance): (",round(distance,1),",",round(rssi.distance,1),")")
    print("Distance Error: ", round(rssi.distance-distance,1), ", Params (n,A): ",np.array([[round(n,1),round(A,1)]]))

    return params, True, rssi




def inRangeCheck(val,valRange):
    if val < valRange[0] or val > valRange[1]:
        return False
    return True

def RSSI_to_distance(rssi):
    n = 1.7
    A = -60
    return 10**(-(rssi-A)/(n*10))

def getLoraPairNumber():
    if GlobalVals.SYSID == 1:
        if GlobalVals.TARGET_BALLOON == 2:
            return 1
        if GlobalVals.TARGET_BALLOON == 3:
            return 2
    if GlobalVals.SYSID == 2:
        if GlobalVals.TARGET_BALLOON == 1:
            return 1
        if GlobalVals.TARGET_BALLOON == 3:
            return 3
    if GlobalVals.SYSID == 3:
        if GlobalVals.TARGET_BALLOON == 1:
            return 2
        if GlobalVals.TARGET_BALLOON == 2:
            return 3
    
    print("SOMETHING IS WRONG IN getLoraPairNumber()!!!!!!!!!!!!")
    
    return -1
    
def pairNumberStart_callback(host,port):
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    while True:
        try:        
            s.connect((GlobalVals.HOST,GlobalVals.RSSI_ALLOCATION_SOCKET))
            s.settimeout(GlobalVals.RSSI_TIMEOUT)
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to lora operation allocation....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the lora operation allocation socket. This thread will now stop.")
                with GlobalVals.BREAK_LORA_ALLOCATION_THREAD_MUTEX:
                    GlobalVals.BREAK_LORA_ALLOCATION_THREAD = True
                return 
        break

    while True:
        with GlobalVals.BREAK_LORA_ALLOCATION_THREAD_MUTEX:
            if GlobalVals.BREAK_LORA_ALLOCATION_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.RSSI_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the Lora Allocation receiver socket. This thread will now stop.")
            break
        

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        string_list = extract_str_btw_curly_brackets(data_str)

        if len(string_list) > 0:
            loraAllocation_list = []
            for string in loraAllocation_list:
                received, node_i = stringToLoraAllocation(string)
                
                if received:
                    loraAllocation_list.append(node_i)
            
            idx = 0
            with GlobalVals.LORA_ALLOCATION_UPDATE_MUTEX:
                while idx < len(loraAllocation_list):
                    loraAllocation_update(loraAllocation_list[idx])
                    idx += 1
    s.close()

def loraAllocation_update(new_data,balloon_id):
    GlobalVals.LORA_ALLOCATION = new_data


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

        if GlobalVals.LORA_ALLOCATION != getLoraPairNumber():
            time.sleep(0.1)
            continue
        else:
            waiting = False

        # DataReady = False
        with GlobalVals.NewRSSISocketData_Mutex:
            if GlobalVals.NewRSSISocketData:
                # DataReady = True
                GlobalVals.NewRSSISocketData = False

        # if not DataReady:
        #     time.sleep(0.1)
        #     continue

        
        
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
                    # distance = RSSI_to_distance(filtered_RSSI)
                    with GlobalVals.GPS_UPDATE_MUTEX:
                        gps_all = copy.deepcopy(GlobalVals.GPS_ALL)
                    
                    rssi = RSSI(filtered_RSSI,None,handshakeTime)
                    GlobalVals.RSSI_PARAMS, GlobalVals.RSSI_CALIBRATION_FINISHED,rssi = RSSI_Calibration(rssi,gps_all,GlobalVals.SYSID,GlobalVals.TARGET_BALLOON)
                    distance = rssi.distance

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

def Thread_RSSI_publish():
     
 
    Logger_Socket = [None]*len(GlobalVals.PORT_RSSI)
    Logger_Connection = [None]*len(GlobalVals.PORT_RSSI)

    for i in range(len(GlobalVals.PORT_RSSI)):
        Logger_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Logger_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        Logger_Socket[i].bind((GlobalVals.HOST, GlobalVals.PORT_RSSI[i][findIndexPort()]))
        Logger_Socket[i].settimeout(GlobalVals.SOCKET_TIMEOUT)

        try: 
            Logger_Socket[i].listen(1)
            Logger_Connection[i], addr = Logger_Socket[i].accept()  
            Logger_Connection[i].settimeout(GlobalVals.SOCKET_TIMEOUT) 
            print("Connected to: ",addr,"Port: ",GlobalVals.PORT_RSSI[i][findIndexPort()])
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

                    socketPayload = "{sysID: "+ str(GlobalVals.SYSID) + "; targetPayloadID: "+ str(GlobalVals.TARGET_BALLOON) + "; RSSI_filter: " + str(RSSI_filtered) + "; distance: " + str(distance) + "; time: " + str(RSSI_time) +";}"
                
                    socketPayload = socketPayload.encode("utf-8")
                    # print(socketPayload)
                    for i in range(len(GlobalVals.PORT_RSSI)):
                        try:
                            Logger_Connection[i].sendall(socketPayload)
                        except Exception as e:
                            print("Exception: " + str(e.__class__))
                            print("Error in the logger socket(sending [",i,"]). Now closing thread.")
                            breakThread = True
                            break
        else:
            time.sleep(0.01)

    # if the thread is broken set the global flag 
    if breakThread:
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
    
    # close connection before ending thread 
    for i in range(len(GlobalVals.PORT_RSSI)):
        Logger_Connection[i].close()

def findIndexPort():
    index = 0
    for i in range(len(GlobalVals.REAL_BALLOON)):
        if GlobalVals.REAL_BALLOON[i] != GlobalVals.SYSID:
            if GlobalVals.REAL_BALLOON[i] == GlobalVals.TARGET_BALLOON:
                # print(index)
                # print(i)
                return index
            index = index + 1
    print("SOMETHING IS WRONG!!!")
    return -1

if __name__ == '__main__':

    # get arguments for running the script
    starter = False
    numArgs = len(sys.argv)
    
    starter = False
    
    # use the third argument as the com port 

    if numArgs >= 3:
        GlobalVals.PORT = sys.argv[2]
    else:
        GlobalVals.PORT = get_port('Lora')
    print('PORT: '+ GlobalVals.PORT)
  
    
    if numArgs >=4:
        GlobalVals.TARGET_BALLOON = int(sys.argv[3])
    
    print('Target Balloon RSSI: ', GlobalVals.TARGET_BALLOON)
    
    if numArgs >=5:
        GlobalVals.SYSID = int(sys.argv[4])

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

    print("Port GPS: ",GlobalVals.PORT_GPS[findIndexPort()])

    
    RSSI_Thread = Thread(target = Thread_RSSI_publish, args = ())
    RSSI_Thread.start()
    # print("Port rssi:",GlobalVals.PORT_RSSI[i][findIndexPort()])

    GPSThread = Thread(target=gps_callback, args=(GlobalVals.HOST,GlobalVals.PORT_GPS[findIndexPort()]))
    GPSThread.start()

    # run the main function until something goes wrong 
    try:
        main(not starter)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")
        
    if RSSI_Thread.is_alive():
        with GlobalVals.EndRSSISocket_Mutex:
            GlobalVals.EndRSSISocket = True
        RSSI_Thread.join()

    if GPSThread.is_alive():
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_THREAD = True
        GPSThread.join()


