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
from navpy import lla2ned
import csv
from common import *


def positionENU(gps,gps_ref):
    pos_ned = lla2ned(gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt).reshape(3,1)
    pos_enu = np.dot(GlobalVals.C_NED_ENU,pos_ned)
    return pos_enu

def distance2D(args):
    gps1 = args[0]
    gps2 = args[1]
    gps_ref = GlobalVals.GPS_REF
    
    pos1_ned = lla2ned(gps1.lat, gps1.lon, gps1.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt)
    pos2_ned = lla2ned(gps2.lat, gps2.lon, gps2.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt)

    pos1_enu = np.dot(GlobalVals.C_NED_ENU,pos1_ned)
    pos2_enu = np.dot(GlobalVals.C_NED_ENU,pos2_ned)

    distance = np.array([math.sqrt((pos1_ned[0]-pos2_ned[0])**2), math.sqrt((pos1_ned[1]-pos2_ned[1])**2)])

    if len(args)==3:
        distance_rssi = args[2]
        distance = distance_rssi/np.linalg.norm(pos1_ned-pos2_ned) * distance
    
    return distance

def rssi_update(new_data):
    GlobalVals.RSSI = new_data

def gps_update(new_data):
    i = sysID_to_index(new_data.sysID)
    GlobalVals.GPS_ALL[i-1] = new_data

def imu_update(new_data):
    i = sysID_to_index(new_data.sysID)
    GlobalVals.IMU_ALL[i-1] = new_data

#{|SYSTEM_ID: 1; EPOCH: 1615999399811; ACCELERATION: -0.237756,0.271728,9.774710; GYRO: 0.003350,-0.001536,0.004171; MAGNETIC_VECTOR: -0.068429,0.860688,0.786252; RAW_QT: 0.545915,0.010954,-0.008901,-0.837722; EULER_321: 1.539973,0.494716,-113.811455}
def stringToIMU(raw_data):
    try:
        raw_data.index("SYSTEM_ID:")
        raw_data.index("EPOCH:")
        raw_data.index("ACCELERATION:")
        raw_data.index("GYRO:")
        raw_data.index("MAGNETIC_VECTOR:")
        raw_data.index("RAW_QT:")
        raw_data.index("EULER_321:")

    except ValueError:
        
        return False, IMU()

    imu_i = IMU()
    try:
        imu_i.sysID = int(extract_string_data("SYSTEM_ID: ",";",raw_data))
        imu_i.epoch = float(extract_string_data("EPOCH: ",";",raw_data))
        imu_i.accel = convert_to_array(extract_string_data("ACCELERATION: ",";",raw_data))
        imu_i.mag_vector = convert_to_array(extract_string_data("MAGNETIC_VECTOR: ",";",raw_data))
        imu_i.raw_qt = convert_to_array(extract_string_data("RAW_QT: ",";",raw_data))
        imu_i.euler = convert_to_array(extract_string_data("EULER_321: ",";",raw_data))
        imu_i.gyros = convert_to_array(extract_string_data("GYRO: ",";",raw_data))

        return True, imu_i

    except ValueError:

        return False, IMU()

def stringToGPS(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")

    except ValueError:
        
        return False, GPS()

    gps_i = GPS()

    try:
        gps_i.sysID = int(extract_string_data("'system': ",";",raw_data))
        gps_i.alt = float(extract_string_data("'altitude': ",";",raw_data))
        gps_i.lat = float(extract_string_data("'latitude': ",";",raw_data))
        gps_i.lon = float(extract_string_data("'longitude': ",";",raw_data))
        gps_i.epoch = float(extract_string_data("'time': ","}",raw_data))

        return True, gps_i

    except ValueError:

        return False, GPS()

def stringToRSSI(raw_data):
    try:
        raw_data.index("RSSI_filter:")
        raw_data.index("distance:")
        raw_data.index("time:")

    except ValueError:
        
        return False, GPS()

    rssi_i = GPS()

    try:
        rssi_i.rssi_filtered = int(extract_string_data("RSSI_filter: ",";",raw_data))
        rssi_i.distance = float(extract_string_data("distance: ",";",raw_data))
        rssi_i.epoch = float(extract_string_data("time: ",";",raw_data))

        return True, rssi_i

    except ValueError:

        return False, RSSI()

def gps_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(GlobalVals.GPS_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the GPS socket. This thread will now stop.")
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_THREAD = True
        return 

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
            print('Blank msg!')
        else:
            data_str = data_bytes.decode('utf-8')
            
            string_list = []
            iterator = 0
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

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
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
            data_bytes = s.recv(GlobalVals.IMU_BUFFER)
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("There was an error starting the IMU receiver socket. This thread will now stop.")
            break

        if len(data_bytes) == 0:
            print('Blank msg!')
        else:
            data_str = data_bytes.decode('utf-8')
            
            string_list = []
            iterator = 0
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

    try:        
        s.connect((host,port))
        s.settimeout(GlobalVals.RSSI_TIMEOUT)
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("There was an error starting the RSSI socket. This thread will now stop.")
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX:
            GlobalVals.BREAK_RSSI_THREAD = True
        return 

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
            print('Blank msg!')
        else:
            data_str = data_bytes.decode('utf-8')
            
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

if __name__ == '__main__':

    numArgs = len(sys.argv)

    if numArgs == 2:
        GlobalVals.SYSID = sys.argv[1]


    GPSThread = Thread(target=gps_callback, args=(GlobalVals.HOST,GlobalVals.PORT_GPS))
    GPSThread.start()

    IMUThread = Thread(target=imu_callback, args = (GlobalVals.HOST,GlobalVals.PORT_IMU))
    IMUThread.start()

    RSSIThread = Thread(target=distanceRSSI_callback, args = (GlobalVals.HOST, GlobalVals.PORT_RSSI))
    RSSIThread.start()

    sysID = GlobalVals.SYSID

    # Update anchor list
    for i in range(len(GlobalVals.ANCHOR)):
        if GlobalVals.ANCHOR[i] == sysID:
            GlobalVals.ANCHOR = np.delete(GlobalVals.ANCHOR,i,None) 

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
        os.makedirs("datalog")
    except FileExistsError:
        pass

    file_name = "datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ekf.csv"

    time.sleep(2)
    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['px','py','pz',])
    

        while True:
            gps = GlobalVals.GPS_ALL[sysID-1]
            imu = GlobalVals.IMU_ALL[sysID-1]
            rssi = GlobalVals.RSSI

            # if imu_prev_time == 0:
            #     pass
            # else:
            #     dt = (imu.epoch - imu_prev_time)/1000
            #     imu_prev_time = imu.epoch

            # if gps.epoch == 0 or imu.epoch == 0:
            #     continue
            # print(imu.epoch)
            # print(gps.epoch)
            anchor_position = []
            anchor_distance = []
            for i in GlobalVals.ANCHOR:
                anchor_position.append(positionENU(GlobalVals.GPS_ALL[i-1],GlobalVals.GPS_REF))
                if i not in GlobalVals.REAL_BALLOON:
                    anchor_distance.append(distance2D([gps, GlobalVals.GPS_ALL[i-1]]))
                else:
                    anchor_distance.append(distance2D([gps, GlobalVals.GPS_ALL[i-1]],rssi.distance))

            accel   = imu.accel
            gyros   = imu.gyros
            magVec  = imu.mag_vector

            # Rotate the coordinates 
            accel   = np.dot(C,accel)
            # gyros   = np.dot(C,gyros)
            # magVec  = np.dot(C,magVec) 

            # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
            IMU = np.concatenate((gyros,accel,magVec))
            ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
            ## The sampling time is based on that of IMU
            timeGPS_IMU_diff = imu.epoch - gps.epoch
            timeRSSI_IMU_diff = imu.epoch - rssi.epoch
            # print("time_diff: ",time_diff)
            if timeGPS_IMU_diff > 2*dt:   
                GPS_data = np.array([])
            else:
                GPS_data = np.dot(GlobalVals.C_NED_ENU,lla2ned( gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt)).reshape(3,1)   # ENU ?


            if timeRSSI_IMU_diff >= 2*dt:
                Dis = np.array([])
            else:
                Dis = np.array([])  # 2D distance !!! Why does it have 3 elements
            
            if Q_Xsens:
                q_sensor = imu.raw_qt.reshape(4)
                
            node = EKF(settings,dt,node,IMU,anchor_position,GPS_data,anchor_distance,Q_Xsens,q_sensor) # EKF

            time.sleep(dt)

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
            GlobalVals.BREAK_RSSI_THREAD = True:
        RSSIThread.join()


