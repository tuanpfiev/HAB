import socket
import struct
import time
from threading import Thread
import threading
from _thread import *
import GlobalVals
import CustMes 
import NetworkManager

import sys
sys.path.insert(1,'../utils')
from common import *
from common_class import *
import copy



def gps_update(new_data):
    GlobalVals.EKF_GPS_ALL = new_data

def rssi_update(new_data):
    GlobalVals.RSSI_ALL = new_data
    
def findTargetPayloadID(index):
    for i in range(len(GlobalVals.REAL_BALLOON)):
        if GlobalVals.REAL_BALLOON[i] == GlobalVals.SYSTEM_ID:
            index = index + 1
        return GlobalVals.REAL_BALLOON[index]
#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
def RSSI_LoggerSocket(host,port,index):

    # set up socket
    while True:
        try:
            socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
            socket_logger.connect((host,port))
            socket_logger.settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT)
            
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to RSSI....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the RSSI Logger socket. This thread will now stop.")
                with GlobalVals.BREAK_RSSI_LOGGER_THREAD_MUTEX[index]:
                    GlobalVals.BREAK_RSSI_LOGGER_THREAD[index] = True
                return 
        break
    print('Connected to RSSI[',index,'], Port: ',port,'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    # intialize variables 
    bufferRead = 1024

    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_RSSI_LOGGER_THREAD_MUTEX[index]:
            if GlobalVals.BREAK_RSSI_LOGGER_THREAD[index]:
                break

        # read the socket 
        try:
            data_bytes = socket_logger.recv(bufferRead)
        except:
            print("Connection error.")
            break
        
        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = extract_str_btw_curly_brackets(data_str)
        
        if len(string_list) > 0:
            rssi_list = []
            for string in string_list:
                received, rssi_i = stringToRSSI(string)
                if received:
                    rssi_list.append(rssi_i)

            idx = 0
            with GlobalVals.RSSI_UPDATE_MUTEX[index]:
                while idx < len(rssi_list):
                    rssi_update(rssi_list[idx])
                    idx += 1
            
            # with GlobalVals.RSSI_UPDATE_MUTEX[index]:
                rssi_i = copy.deepcopy(GlobalVals.RSSI_ALL)
                # print(rssi_i)
        
            RSSI_Data = CustMes.MESSAGE_RSSI()
            RSSI_Data.FilteredRSSI = rssi_i.rssi_filtered
            RSSI_Data.Distance = rssi_i.distance
            RSSI_Data.Epoch = rssi_i.epoch
            RSSI_Data.SystemID = GlobalVals.SYSTEM_ID
            RSSI_Data.TargetPayloadID = int(findTargetPayloadID(index))

            # add data to the gps buffer 
            with GlobalVals.RSSI_DATA_BUFFER_MUTEX:
                GlobalVals.RSSI_DATA_BUFFER.append(RSSI_Data)

            # # set the flag for the data 
            with GlobalVals.RECIEVED_RSSI_LOCAL_DATA_MUTEX: # 2 nodes?
                GlobalVals.RECIEVED_RSSI_LOCAL_DATA = True

            with GlobalVals.RSSI_ALLOCATION_MUTEX:
                print("UPDATE RSSI ALLOCATION MATRIX LOCALLYYYY")
                print(GlobalVals.RSSI_ALLOCATION)
                GlobalVals.RSSI_ALLOCATION[GlobalVals.SYSTEM_ID-1][RSSI_Data.TargetPayloadID-1] = True

            # send GPS data to other balloons 
            RSSI_Packet = CustMes.MESSAGE_FRAME()
            RSSI_Packet.SystemID = GlobalVals.SYSTEM_ID
            RSSI_Packet.MessageID = 7
            RSSI_Packet.TargetID = 0
            RSSI_Packet.Payload = RSSI_Data.data_to_bytes()
            NetworkManager.sendPacket(RSSI_Packet)
            # print(RSSI_Data)
            # print("***************************")

        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(1)  

    socket_logger.close()
    return 


def RSSI_Distributor():

    Distro_Socket = [None]*GlobalVals.N_RSSI_NODE_PUBLISH
    Distro_Connection = [None]*GlobalVals.N_RSSI_NODE_PUBLISH
    for i in range(GlobalVals.N_RSSI_NODE_PUBLISH):
        # start socket 

        Distro_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Distro_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
        Distro_Socket[i].bind((GlobalVals.HOST, GlobalVals.RSSI_DISTRO_SOCKET[i]))
        Distro_Socket[i].settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT)
        

        # Wait for connection on the distro socket 
        try:
            Distro_Socket[i].listen(1) 
            Distro_Connection[i], addr = Distro_Socket[i].accept()  
            Distro_Connection[i].settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT) 
            print("Logger[",i,"] Connected to ", addr)                                            
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error in the RSSI_Distributor[",i,"] logger socket. Now closing thread.")
            with GlobalVals.BREAK_RSSI_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_RSSI_DISTRO_THREAD = True
            return 0
  
    
    source1 = False
    source2 = False
    breakThread = False

    while True:

        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.BREAK_RSSI_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_RSSI_DISTRO_THREAD:
                break

        # check if local GPS data has been recived 
        with GlobalVals.RECIEVED_RSSI_LOCAL_DATA_MUTEX:
            if GlobalVals.RECIEVED_RSSI_LOCAL_DATA:
                source1 = True
                GlobalVals.RECIEVED_RSSI_LOCAL_DATA = False
        
        # check if GPS data from the radio has been recieved 
        with GlobalVals.RECIEVED_RSSI_RADIO_DATA_MUTEX:
            if GlobalVals.RECIEVED_RSSI_RADIO_DATA:
                source2 = True
                GlobalVals.RECIEVED_RSSI_RADIO_DATA = False
        
        # if no data has been recieved sleep and loop
        if not source1 and not source2:
            time.sleep(0.1)
            continue
        else:
            source1 = False
            source2 = False

        with GlobalVals.RSSI_DATA_BUFFER_MUTEX:
            while len(GlobalVals.RSSI_DATA_BUFFER) > 0:

                # get the GPS data
                RSSI_Data = GlobalVals.RSSI_DATA_BUFFER.pop(0)
                distance = RSSI_Data.Distance
                filteredRSSI = RSSI_Data.FilteredRSSI
                targetPayloadID = RSSI_Data.TargetPayloadID
                epoch = RSSI_Data.Epoch
                systemID = RSSI_Data.SystemID

                messageStr = "{sysID: " + str(systemID) + "; time: " + str(epoch) + "; RSSI_filter: " + str(filteredRSSI) + "; distance: " + str(distance) + "; targetPayloadID: " + str(targetPayloadID) + ";}"
                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                for i in range(GlobalVals.N_RSSI_NODE_PUBLISH):
                    try:
                        Distro_Connection[i].sendall(messageStr_bytes)
                        # print("Sending RSSI:",messageStr)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error when sending to RSSI Distro_Connection[",i,"]. Now closing thread.")
                        breakThread = True
                        break
                
    for i in range(GlobalVals.N_RSSI_NODE_PUBLISH):
        Distro_Connection[i].close()


def pairIndex(pairNum):
    i = 0
    j = 0
    if pairNum == 1:
        i = 1
        j = 2
    
    if pairNum == 2:
        i = 2
        j = 3
    
    if pairNum == 3:
        i = 3
        j = 1
    
    return i-1,j-1

def resetRSSI_Allocation(pairNum):
    i,j = pairIndex(pairNum)

    GlobalVals.RSSI_ALLOCATION[i][j] = False
    GlobalVals.RSSI_ALLOCATION[j][i] = False

def resetRSSI_Allocation():
    for i in range(len(GlobalVals.RSSI_ALLOCATION)):
        for j in range(len(GlobalVals.RSSI_ALLOCATION[0])):
            GlobalVals.RSSI_ALLOCATION[i][j] = False

def checkRSSI_Allocation(pairNum):
    i,j = pairIndex(pairNum)

    if GlobalVals.RSSI_ALLOCATION[i][j] and GlobalVals.RSSI_ALLOCATION[j][i]:
        return True
    else:
        return False


def RSSI_AllocationDistributor():
    # print("RSSI ALLOCATION DISTRIBUTOR 1")
    Distro_Socket = [None]*len(GlobalVals.RSSI_ALLOCATION_DISTRO_SOCKET)
    Distro_Connection = [None]*len(GlobalVals.RSSI_ALLOCATION_DISTRO_SOCKET)
    for i in range(len(GlobalVals.RSSI_ALLOCATION_DISTRO_SOCKET)):
        # start socket 

        Distro_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Distro_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
        Distro_Socket[i].bind((GlobalVals.HOST, GlobalVals.RSSI_ALLOCATION_DISTRO_SOCKET[i]))
        Distro_Socket[i].settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT)
        

        # Wait for connection on the distro socket 
        try:
            Distro_Socket[i].listen(1) 
            Distro_Connection[i], addr = Distro_Socket[i].accept()  
            Distro_Connection[i].settimeout(GlobalVals.RSSI_LOGGER_SOCKET_TIMEOUT) 
            print("RSSI Allocation Logger[",i,"] Connected to ", addr)                                            
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error in the RSSI_Allocation Distributor[",i,"] logger socket. Now closing thread.")
            with GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD = True
            return 0
    
    nextPair = 1
    while True:    
        # print("RSSI ALLOCATION DISTRIBUTOR 2")

        with GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_RSSI_ALLOCATION_DISTRO_THREAD:
                break
        with GlobalVals.RSSI_ALLOCATION_MUTEX:
            if GlobalVals.SYSTEM_ID == 1:
                nextPair = GlobalVals.NEXT_PAIR
            else:
                with GlobalVals.RSSI_DATA_ALLOCATION_BUFFER_MUTEX:
                    if len(GlobalVals.RSSI_DATA_ALLOCATION_BUFFER) > 0:
                        RSSI_DataAllocation = GlobalVals.RSSI_DATA_ALLOCATION_BUFFER.pop(0)
                        nextPair = RSSI_DataAllocation.Pair

        messageStr = "{'pair': " + str(nextPair) +";}"
        messageStr_bytes = messageStr.encode('utf-8')

        # print('Allocated pair: '+messageStr)
        # send the message 
        for i in range(GlobalVals.N_RSSI_NODE_PUBLISH):
            try:
                Distro_Connection[i].sendall(messageStr_bytes)
            except Exception as e:
                print("Exception: " + str(e.__class__))
                print("Error when sending to RSSI Allocation Distro_Connection[",i,"]. Now closing thread.")
                breakThread = True
                time.sleep(2)
                break
        
    for i in range(GlobalVals.N_RSSI_NODE_PUBLISH):
        Distro_Connection[i].close()


def getPairAllocation():
    with GlobalVals.RSSI_ALLOCATION_MUTEX:
        nextPairStatus = checkRSSI_Allocation(GlobalVals.NEXT_PAIR)
        if nextPairStatus:
            if GlobalVals.NEXT_PAIR == GlobalVals.N_REAL_BALLOON:
                GlobalVals.NEXT_PAIR = 1
            else:
                GlobalVals.NEXT_PAIR = GlobalVals.NEXT_PAIR + 1
            resetRSSI_Allocation()