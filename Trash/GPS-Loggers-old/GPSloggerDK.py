# Import DroneKit-Python
from __future__ import print_function
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions
import math
import time
import socket
from threading import Thread, Lock
#from typing_extensions import final 

# Coms Constants 
connection_string_sim = "tcp:127.0.0.1:5760"        # MAVlink Connection string for simulation 
#connection_string = "/dev/ttyUSB0"                 # MAVlink USB UART connection string 
connection_string = "/dev/ttyS0"                    # MAVlink GPIO UART connection string 
BaudRate = 57600                                    # Baud rate for the serial connecting to the autopilot 
HOST = ''                                  # Local host address
PORT_SIM = 5001                                     # Arbitrary non-privileged port for connecting to path simulation script
PORT_LOC = 5002                                     # Arbitrary non-privileged port for connecting to localisation script
PORT_RAD = 5003                                     # Arbitrary non-privileged port for connecting to radio network
CONNECT_TIMEOUT = 3                                 # Timeout for socket connections  

# Constant Strings 
LogFileName = "Log.txt"                             # The file name that will be used for the local log 

# Global Arrays 
AltLog = []                                         # Array for logging a pre set number of altitude values 
AltLength = 10                                      # Max number of alt values recorded
PreAlt = 0                                          # Previous altitude  
PreTime = 0                                 

# Flags
Connected_Socket = False                            # Flag for when a device has connected to the socket (True = connected) 
ThreadMade = False                                  # Flag indicating i the network thread started (true = thread has started)
StopThread = False                                  # Flag used to gracefully stop the network thread (true = stop thread) 
Simulation = True                                   # Simulation flag, if true the code will connect to the simulation
ReadyToSend = False                                 # Flag indicating if the log string is ready to send over the network 
ThreadBroken = False                                # Global flag indicating if the thread has broken 
ListenerBroken = False                              # Global flag indicating if the listener has broken 
ClearAltLog = True                                  # Flag for clearing the AltLog
SimIsConnected = False                              # Listener status flag indicating that the sim is connected 
LocIsConnected = False                              # Listener status flag indicating that the loc is connected 
RadIsConnected = False                              # Listener status flag indicating that the rad is connected 

# Global variables 
LogString = ""                                      # String for sharing the log data with the networking thread 

# Mutexes 
Socket_Mutex = Lock()                               # A mutex for ReadyToSend and LogString
Broken_Mutex = Lock()                               # A mutex for acessing the global broken flags 
ListenerStatus_Mutex = Lock()                       # A mutex for acessing the global listener status flags  

# System Time listener, used to update GPS info  
def GPSInfo(self, name, msg):

    # Global values 
    global LogString 
    global ReadyToSend
    global ListenerBroken
    global ThreadBroken
    global AltLog
    global AltLength
    global PreAlt
    global ClearAltLog
    global PreTime 

    # Default values 
    Alt = 0
    Lon = 0
    Lat = 0
    TimeGPS = 0

    # Try to get values from vehicle 
    try:
        Alt = vehicle.location.global_frame.alt
        Lon = vehicle.location.global_frame.lon
        Lat = vehicle.location.global_frame.lat
        TimeGPS = msg.time_unix_usec
    except Exception as e:
        print("Exception: ", e.__class__)
        with Broken_Mutex:
            ListenerBroken = True 
        return 
    
    # get current time in seconds 
    PointTime = math.floor(TimeGPS / 1000000) 

    # if log needs to be cleared or its the first run then clear it
    if ClearAltLog:
        AltLog.clear() 
        ClearAltLog = False  
        PreAlt = Alt
        PreTime = PointTime
    
    # get the length of the ALtLog
    LogLength = len(AltLog) 

    # if 1 or more seconds have passed calculate the new ascent rate 
    if PointTime - PreTime != 0:
        AltLog.append(Alt - PreAlt)
        PreAlt = Alt
        PreTime = PointTime

        # If the log is bigger then the limit remove the first value
        if LogLength > AltLength:
            AltLog.pop(0)

    # Average the values in AltLog
    avAlt = 0 
    if LogLength != 0: 
        for x in AltLog:
            avAlt = avAlt + x
        avAlt = avAlt / LogLength

    # create string to send over the network 
    tempStr = "{'altitude': " + str(Alt) + "; 'latitude': " + str(Lat) + "; 'longitude': " + str(Lon) + "; 'time': " + str(TimeGPS) + "; 'ascent_rate': " + str(avAlt) + "}"
    print(tempStr)

    # Check if the network thread hasn't broken  
    canSend = False
    with Broken_Mutex:
        if not ThreadBroken:
            canSend = True 
    
    # if the network thread is good let it know it can send
    if canSend:
        with Socket_Mutex:
            LogString = tempStr
            ReadyToSend = True 
    
    # Write log string to log file 
    try:
        fileObj = open(LogFileName, "a")
    except Exception as e:
        print("Exception: ", e.__class__)
        with Broken_Mutex:
            ListenerBroken = True 
        return 
    
    fileObj.write(tempStr)
    fileObj.write("\n")
    fileObj.close
        


# Socket Thread 
def NetworkThread():
    
    global Connected_Socket
    global HOST
    global PORT_LOC
    global PORT_RAD
    global PORT_SIM
    global LogString
    global ReadyToSend
    global StopThread 
    global ListenerBroken
    global ThreadBroken 
    global CONNECT_TIMEOUT
    global SimIsConnected
    global LocIsConnected
    global RadIsConnected

    socket_sim = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # creates socket using the AF_INET domain and SOCK_STREAM socket type 
    socket_loc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   
    socket_rad = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   

    socket_sim.bind((HOST, PORT_SIM))                                   # binds the socket to the port  
    socket_loc.bind((HOST, PORT_LOC))
    socket_rad.bind((HOST, PORT_RAD))
        
    socket_sim.settimeout(CONNECT_TIMEOUT)                              # set time out for sockets 
    socket_loc.settimeout(CONNECT_TIMEOUT)
    socket_rad.settimeout(CONNECT_TIMEOUT)

    # Wait for connection on the simulation socket 
    try:
        socket_sim.listen(1) 
        connect_sim, addr_sim = socket_sim.accept()  
        connect_sim.settimeout(1) 
        print("Simulation connected by ", addr_sim)                                            
        with ListenerStatus_Mutex:
            SimIsConnected = True
    except:
        print("Issue with Sim connection. Will continue with other connections.")
        with ListenerStatus_Mutex:
            SimIsConnected = False
        
    # Wait for connection on the localisation socket 
    try:
        socket_loc.listen(1)     
        connect_loc, addr_loc = socket_loc.accept()  
        connect_loc.settimeout(1)
        print("Localisation connected by ", addr_loc)                                          
        with ListenerStatus_Mutex:
            LocIsConnected = True
    except:
        print("Issue with Loc connection. Will continue with other connections.")
        with ListenerStatus_Mutex:
            LocIsConnected = False
    
    # Wait for connection on the radio socket 
    try:
        socket_rad.listen(1)    
        connect_rad, addr_rad = socket_rad.accept()
        connect_rad.settimeout(1)    
        print("Radio connected by ", addr_rad)                                         
        with ListenerStatus_Mutex:
            RadIsConnected = True
    except:
        print("Issue with Rad connection. Will continue with other connections.")
        with ListenerStatus_Mutex:
            RadIsConnected = False
    

    with ListenerStatus_Mutex:
        if RadIsConnected or LocIsConnected or SimIsConnected:
            Connected_Socket = True

    # waits until ready to send, checks every 100ms  
    while True:
        time.sleep(0.1)
        
        with Socket_Mutex:
            
            if ReadyToSend and not StopThread:
                
                with ListenerStatus_Mutex:
                    
                    if SimIsConnected:
                        try:
                            connect_sim.sendall(LogString.encode())
                        except Exception as e:
                            print("Sim connection broken.")
                            print("Exception: ", e.__class__)
                            SimIsConnected = False
                
                    if LocIsConnected:
                        try:
                            connect_loc.sendall(LogString.encode())
                        except Exception as e:
                            print("Loc connection broken.")
                            print("Exception: ", e.__class__)
                            LocIsConnected = False
                
                    if RadIsConnected:
                        try:
                            connect_rad.sendall(LogString.encode())
                        except Exception as e:
                            print("Loc connection broken.")
                            print("Exception: ", e.__class__)
                            RadIsConnected = False
                
                    if not SimIsConnected and not LocIsConnected and not RadIsConnected:
                        with Broken_Mutex:
                            ThreadBroken = True
                        ReadyToSend = False
                        break

            if StopThread:
                with Broken_Mutex:
                    ThreadBroken = True
                break
        
        with Broken_Mutex:
            if ListenerBroken:
                ThreadBroken = True
                break
    
    try:
        connect_sim.close()
    except:
        pass

    try:
        connect_loc.close()
    except:
        pass

    try:
        connect_rad.close()
    except:
        pass

    return

# the main file code starts here 
if __name__ == '__main__':

    VehicleMade = False
    ThreadMade = False

    # Main Try block
    try:

        # Connect to the Vehicle.
        if Simulation:
            print("Connecting to vehicle on: %s" % connection_string_sim)
        else:
            print("Connecting to vehicle on: %s" % connection_string)
    
        try:
            if Simulation:
                vehicle = connect(connection_string_sim, wait_ready=True)
            else: 
                vehicle = connect(connection_string, wait_ready=True, baud=BaudRate)
            VehicleMade = True
        except Exception as e:
            print("Error connecting to autopilot.")
            print("Exception: ", e.__class__)
            VehicleMade = False
            raise e
        
        try:
            Thread_Networking= Thread(target=NetworkThread, args=())
            Thread_Networking.start()
            ThreadMade = True
        except Exception as e:
            print("Exception: ", e.__class__)
            print("ERROR: Network thread was unable to start.") 
            ThreadMade = False
            raise e

        # Wait until socket is set up 
        while not Connected_Socket:
            print("Waiting for socket connection...")
            time.sleep(1)
            with Broken_Mutex:
                if ThreadBroken:
                    print("Something went wrong in the networking thread and it has stoppped.")
                    break

        with Broken_Mutex:
            if not ThreadBroken: 
                print ("Socket Connected.")

        # Wait until the GPS has a fix 
        while vehicle.gps_0.fix_type != 3:
            print("Waiting for 3D GPS fix...")
            time.sleep(1)
        print("GPS has 3D fix.")

        # Add listener for system time changes
        # - Time is updated using the GPS
        # - Therefore any time updates will come with GPS updates 
        print("Adding system time listener.")
        vehicle.add_message_listener('SYSTEM_TIME',GPSInfo)

        # loop until exception (mainly keyboard exception) or until the thread or listener is broken
        while True:
            time.sleep(0.1)
            with Broken_Mutex:
                if ThreadBroken:
                    break
                if ListenerBroken:
                    break
    
    # Main exception block 
    except Exception as e:

        # General exception handling 
        print("Exception: ", e.__class__)

    # Finally block for closing thread and vehicle 
    finally:
        
        print("Closing program...")

        # Close vehicle object
        if VehicleMade:
            vehicle.close()
        
        # Close the networking thread
        if ThreadMade:
            with Socket_Mutex:
                StopThread = True
            Thread_Networking.join()
