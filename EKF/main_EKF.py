from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 
from class_def import *
import socket, time, os, sys
from threading import Thread
import GlobalVals
import math


sys.path.insert(1,'../utils/')
from navpy import lla2ned, ned2lla
import csv
from common import *
from common_class import *


def enu2lla(enu, gps_ref, latlon_unit='deg', alt_unit='m', model='wgs84'):
    ned = np.dot(GlobalVals.C_ENU_NED,enu)
    lla = ned2lla(ned,gps_ref.lat, gps_ref.lon, gps_ref.alt, latlon_unit, alt_unit, model)
    return lla


def distance2D(args):
    gps1 = args[0]
    gps2 = args[1]
    gps_ref = GlobalVals.GPS_REF
    
    pos1_enu = positionENU(gps1,gps_ref)
    pos2_enu = positionENU(gps2,gps_ref)

    distance = np.array([math.sqrt((pos1_enu[0]-pos2_enu[0])**2+(pos1_enu[1]-pos2_enu[1])**2)])

    if len(args)==3:
        distance_rssi = args[2]
        distance = distance_rssi/np.linalg.norm(pos1_enu-pos2_enu) * distance
    
    return distance

def rssi_update(new_data):
    GlobalVals.RSSI = new_data
    # print(GlobalVals.RSSI.epoch)

def gps_update(new_data):
    i = sysID_to_index(new_data.sysID)
    GlobalVals.GPS_ALL[i-1] = new_data

def imu_update(new_data):
    i = sysID_to_index(new_data.sysID)
    GlobalVals.IMU_ALL[i-1] = new_data


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
        iterator = data_str.find('{')
        while data_str.find('}', iterator) != -1:
            substring_end = data_str.find('}', iterator)
            string_list.append(data_str[iterator:substring_end + 1])
            iterator = substring_end + 1
        
        if len(string_list) > 0:
            gps_list = []
            for string in string_list:
                received, gps_i = stringToGPS(string)
                if received:
                    gps_list.append(gps_i)
            
            idx = 0
            while idx < len(gps_list):
                gps_update(gps_list[idx])
                idx += 1

    s.close()

def imu_callback(host,port):

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:        
        s.bind((GlobalVals.HOST,GlobalVals.PORT_IMU))
        s.settimeout(GlobalVals.IMU_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the IMU socket. This thread will now stop.")
        with GlobalVals.BREAK_IMU_THREAD_MUTEX:
            GlobalVals.BREAK_IMU_THREAD = True
        return 

    while True:
        with GlobalVals.BREAK_IMU_THREAD_MUTEX:
            if GlobalVals.BREAK_IMU_THREAD:
                break

        try:
            data_bytes,addr = s.recvfrom(GlobalVals.IMU_BUFFER)
            # print('Connect to: ',addr)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the IMU receiver socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        
        string_list = []
        iterator = data_str.find('{')
        while data_str.find('}', iterator) != -1:
            substring_end = data_str.find('}', iterator)
            string_list.append(data_str[iterator:substring_end + 1])
            iterator = substring_end + 1
        
        if len(string_list) > 0:
            imu_list = []
            for string in string_list:
                received, imu_i = stringToIMU(string)
                if received:
                    imu_list.append(imu_i)
            
            idx = 0
            while idx < len(imu_list):
                imu_update(imu_list[idx])
                idx += 1
    s.close()


def distanceRSSI_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    while True:
        try:        
            s.connect((host,port))
            s.settimeout(GlobalVals.RSSI_TIMEOUT)
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to EKF....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the RSSI socket. This thread will now stop.")
                with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
                    GlobalVals.BREAK_RSSI_THREAD = True
                return 
        break

    while True:
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            if GlobalVals.BREAK_RSSI_THREAD:
                break

        try:
            data_bytes = s.recv(GlobalVals.RSSI_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the RSSI receiver socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        # print('data RSSI: ',data_str)
        string_list = []
        iterator = data_str.find('{')
        while data_str.find('}', iterator) != -1 and iterator != -1:
            substring_end = data_str.find('}', iterator)
            string_list.append(data_str[iterator:substring_end + 1])
            iterator = substring_end + 1
        
        if len(string_list) > 0:
            rssi_list = []
            for string in string_list:
                received, rssi_i = stringToRSSI(string)
                if received:
                    rssi_list.append(rssi_i)
            
            idx = 0
            while idx < len(rssi_list):
                rssi_update(rssi_list[idx])
                idx += 1
    s.close()


def LLA_EKF_Distributor():
    # start socket 
    Distro_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Distro_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
    Distro_Socket.bind((GlobalVals.HOST, GlobalVals.EKF_GPS_DISTRO_SOCKET))
    Distro_Socket.settimeout(GlobalVals.EKF_GPS_LOGGER_SOCKET_TIMEOUT)
    

    # Wait for connection on the distro socket 
    try:
        Distro_Socket.listen(1) 
        Distro_Connection, addr = Distro_Socket.accept()  
        Distro_Connection.settimeout(GlobalVals.EKF_GPS_LOGGER_SOCKET_TIMEOUT) 
        print("Logger Connected to ", addr)                                            
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD = True
        return

    breakThread = False

    while True:
        if breakThread:
            break

        with GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD:
                break

        with GlobalVals.LLA_EKF_BUFFER_MUTEX:
            while len(GlobalVals.LLA_EKF_BUFFER)>0:
                llaEKF = GlobalVals.LLA_EKF_BUFFER.pop(0)

                messageStr = "{'system': " + str(llaEKF.sysID) + "; 'altitude': " + str(llaEKF.alt) + "; 'latitude': " + str(llaEKF.lat) + "; 'longitude': " + str(llaEKF.lon) + "; 'time': " + str(llaEKF.epoch) + "}"
                messageStr_bytes = messageStr.encode('utf-8')
                # print(messageStr_bytes)
                # print("*******************************")
                try:
                    # Thread(target=Threaded_Client, args=([Distro_Connection,messageStr_bytes]))
                    # start_new_thread(Threaded_Client,(Distro_Connection,messageStr_bytes))
                    Distro_Connection.sendall(messageStr_bytes)
                except Exception as e:
                    print("Exception: " + str(e.__class__))
                    print("Error in the EKF logger socket. Now closing thread.")
                    breakThread = True
                    break
    Distro_Connection.close()

if __name__ == '__main__':

    numArgs = len(sys.argv)

    if numArgs == 2:
        GlobalVals.SYSID = int(sys.argv[1])


    GPSThread = Thread(target=gps_callback, args=(GlobalVals.HOST,GlobalVals.PORT_GPS))
    GPSThread.start()

    IMUThread = Thread(target=imu_callback, args = (GlobalVals.HOST,GlobalVals.PORT_IMU))
    IMUThread.start()

    RSSIThread = Thread(target=distanceRSSI_callback, args = (GlobalVals.HOST, GlobalVals.PORT_RSSI))
    RSSIThread.start()


    LLA_EKF_DistributorThread = Thread(target = LLA_EKF_Distributor, args = ())
    LLA_EKF_DistributorThread.start()

    sysID = GlobalVals.SYSID

    print("WAITING for the GPS & RSSI data. Calculation has NOT started yet...")
    while True:
        # if not GlobalVals.RSSI and checkAllGPS(GlobalVals.GPS_ALL):
        print(checkAllGPS(GlobalVals.GPS_ALL))
        print(GlobalVals.RSSI.epoch != 0.0)
        if GlobalVals.RSSI.epoch != 0.0 and checkAllGPS(GlobalVals.GPS_ALL):
            break

        time.sleep(1)

    print("Calculation loop STARTED!!!")
    # Update anchor list
    for i in range(len(GlobalVals.ANCHOR)):
        if GlobalVals.ANCHOR[len(GlobalVals.ANCHOR)-i-1] == sysID:
            GlobalVals.ANCHOR = np.delete(GlobalVals.ANCHOR,len(GlobalVals.ANCHOR)-i-1,None) 

    ##
    ## Initialization
    ##

    gps_ref = GlobalVals.GPS_REF
    mag0 = GlobalVals.IMU_ALL[sysID-1].mag_vector
    acc0 = GlobalVals.IMU_ALL[sysID-1].accel
    pos0_enu = positionENU(GlobalVals.GPS_ALL[sysID-1], GlobalVals.GPS_REF)

    node = node(mag0,acc0,pos0_enu)

    settings = settings()
    dt = GlobalVals.dt ## The sampling time is based on IMU
    imu_prev_time = 0
    Q_Xsens = True  #If Q_Xsens = False, EKF will solve the Euler angle; 
                #if Q_Xsens = True, EKF will not solve the Euler angle and the angle from sensor output will be used. 
                #In the latter case, the quaternion data should be used
                #For basic experiment, Q_Xsens = False
                #If it is possible, we can run two parallel scripts -- one using Q_Xsens = False and another using Q_Xsens = True
    q_sensor = np.array([]) # variable for quaternion data obtianed from sensor

    C = np.array([[-1,0, 0],[0,-1,0],[0,0,-1]]) # correcting acc

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ekf.csv"

    time.sleep(2)
    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['sysID','x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','roll','pitch','yaw',\
            'gps0_lat','gps0_lon','gps0_alt','gps1_lat','gps1_lon','gps1_alt','gps2_lat','gps2_lon','gps2_alt','gps3_lat','gps3_lon','gps3_alt',\
                'gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z','qt1','qt2','qt3','qt4','epoch',\
                    'magVec1','magVec2','magVec3'])

        while True:
            gps_all = GlobalVals.GPS_ALL
            gps = GlobalVals.GPS_ALL[sysID-1]
            imu = GlobalVals.IMU_ALL[sysID-1]
            rssi = GlobalVals.RSSI
            epoch = time.time()
            timeLocal = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch))
            anchor_position = np.zeros([len(GlobalVals.ANCHOR),2])
            if checkAllGPS(GlobalVals.GPS_ALL):
                for i in range(len(GlobalVals.ANCHOR)):
                    posEN = positionENU(GlobalVals.GPS_ALL[GlobalVals.ANCHOR[i]-1],gps_ref).T
                    anchor_position[i,:]=posEN[0][0:2]   
            
            # Rotate the coordinates 
            accel   = np.dot(C,imu.accel)

            # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
            IMU_i = np.concatenate((accel,imu.gyros,imu.mag_vector))
            ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
            ## The sampling time is based on that of IMU
            timeGPS_IMU_diff = imu.epoch/1000 - gps.epoch       # IMU epoch is in ms
            timeRSSI_IMU_diff = imu.epoch/1000 - rssi.epoch
            # print("time_GPS_diff: ",timeGPS_IMU_diff)
            # print("time_RSSI_diff: ",timeRSSI_IMU_diff)
            
            GPS_data = np.array([])
            if checkGPS(gps) and timeGPS_IMU_diff <= 2*dt:
                GPS_data = positionENU(gps,gps_ref)[0]
                # print(GPS_data)

            anchor_distance = np.array([])
            if checkAllGPS(GlobalVals.GPS_ALL) and timeRSSI_IMU_diff <= 2*dt:
                anchor_distance = np.zeros([len(GlobalVals.ANCHOR),1])           
                for i in range(len(GlobalVals.ANCHOR)):
                    if GlobalVals.ANCHOR[i] not in GlobalVals.REAL_BALLOON:
                        temp = distance2D([gps, GlobalVals.GPS_ALL[i-1]])
                        anchor_distance[i,:] = distance2D([gps, GlobalVals.GPS_ALL[GlobalVals.ANCHOR[i]-1]])
                        
                    else:
                        temp = distance2D([gps, GlobalVals.GPS_ALL[i-1],rssi.distance])
                        anchor_distance[i,:] = distance2D([gps, GlobalVals.GPS_ALL[GlobalVals.ANCHOR[i]-1],rssi.distance])
                # print(anchor_distance)
                # print("===============")

            if Q_Xsens:
                q_sensor = imu.raw_qt.reshape(4)
            print('Time: ',timeLocal)
            node = EKF(settings,dt,node,IMU_i,anchor_position,GPS_data,anchor_distance,Q_Xsens,q_sensor) # EKF
            
            x_h = np.array([node.x_h[:,-1]]).T
            output.writerow([GlobalVals.SYSID, x_h[0][0],x_h[1][0],x_h[2][0],x_h[3][0],x_h[4][0],x_h[5][0],x_h[6][0],x_h[7][0],x_h[8][0],x_h[9][0],node.roll,node.pitch,node.yaw,\
                gps_all[0].lat, gps_all[0].lon, gps_all[0].alt, gps_all[1].lat, gps_all[1].lon, gps_all[1].alt, gps_all[2].lat, gps_all[2].lon, gps_all[2].alt, gps_all[3].lat, gps_all[3].lon, gps_all[3].alt,
                    imu.gyros[0][0],imu.gyros[1][0],imu.gyros[2][0],accel[0][0],accel[1][0],accel[2][0],imu.raw_qt[0][0],imu.raw_qt[1][0],imu.raw_qt[2][0],imu.raw_qt[3][0],epoch,\
                        imu.mag_vector[0][0],imu.mag_vector[1][0],imu.mag_vector[2][0]])
            time.sleep(dt)


            posENU_EKF = np.array([x_h[0][0],x_h[1][0],x_h[2][0]]).T
            llaEKF = enu2lla(posENU_EKF, gps_ref)
            
            with GlobalVals.LLA_EKF_BUFFER_MUTEX:
                GlobalVals.LLA_EKF_BUFFER.append(GPS(sysID, llaEKF[0],llaEKF[1],llaEKF[2],epoch))


            
    # while True:
    #     time.sleep(1)

    if GPSThread.is_alive():
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_THREAD = True
        GPSThread.join()

    if IMUThread.is_alive():
        with GlobalVals.BREAK_IMU_THREAD_MUTEX:
            GlobalVals.BREAK_IMU_THREAD = True
        IMUThread.join()

    if RSSIThread.is_alive():
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_THREAD = True
        RSSIThread.join()

    if LLA_EKF_DistributorThread.is_alive():
        with GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_EKF_GPS_DISTRO_THREAD = True
        LLA_EKF_DistributorThread.join()


