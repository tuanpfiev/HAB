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
        
        return False, RSSI()

    rssi_i = RSSI()

    try:
        temp = extract_string_data("RSSI_filter: ",";",raw_data)
        rssi_i.rssi_filtered = float(extract_string_data("RSSI_filter: ",";",raw_data))
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
        os.makedirs("datalog")
    except FileExistsError:
        pass

    file_name = "datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ekf.csv"

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


