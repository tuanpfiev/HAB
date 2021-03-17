from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 
from class_def import *
import socket, time, os
from threading import Thread


sys.path.insert(1,'../utils')
from navpy import lla2ned
import csv

global buffer, gps_all, imu_all
buffer = 1024
n_balloon = 5
gps_all = np.array([GPS()]*n_balloon)
imu_all = np.array([IMU()]*n_balloon)

def sysID_to_index(sysID: int):
    if sysID == 1:
        return 1
    elif sysID == 2:
        return 2
    elif sysID == 253:
        return 3
    elif sysID == 254:
        return 4
    elif sysID == 255:
        return 5
    else:
        print('SysID should be in the range 1-5')
        os._exit(1)
        return 0

def gps_update(new_data):
    global gps_all
    i = sysID_to_index(new_data.sysID)
    gps_all[i-1] = new_data

def imu_update(new_data):
    global imu_all
    i = sysID_to_index(new_data.sysID)
    imu_all[i-1] = new_data

def extract_string_data(preString,endString,string_data):
    preIndex = string_data.find(preString)
    endIndex = string_data.find(endString, preIndex)
    return string_data[preIndex + len(preString):endIndex]

def convert_to_array(string_data):
    start_parsing = 0
    array = []
    while True:
        comma_index = string_data.find(",",start_parsing)
        if comma_index != -1:
            val = float(string_data[start_parsing:comma_index])
            array.append(val)
            start_parsing = comma_index + 1
        else:
            try:
                val = float(string_data[start_parsing:len(string_data)])
            except:
                break
            array.append(val)
            break
    return array


def stringToIMU(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'time':")
        raw_data.index("'acceleration':")
        raw_data.index("'magneticVector':")
        raw_data.index("'rawQT':")
        raw_data.index("'euler321':")
        raw_data.index("'gyroscope':")

    except ValueError:
        
        return False, IMU()

    imu_i = IMU()
    try:
        imu_i.sysID = int(extract_string_data("'system': ",";",raw_data))
        imu_i.epoch = float(extract_string_data("'time': ",";",raw_data))
        imu_i.accel = convert_to_array(extract_string_data("'acceleration': ",";",raw_data))
        imu_i.mag_vector = convert_to_array(extract_string_data("'magneticVector': ",";",raw_data))
        imu_i.raw_qt = convert_to_array(extract_string_data("'rawQT': ",";",raw_data))
        imu_i.euler = convert_to_array(extract_string_data("'euler321': ",";",raw_data))
        imu_i.gyros = convert_to_array(extract_string_data("'gyroscope': ","}",raw_data))

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


def gps_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(2)
    except:
        pass

    while True:
        try:
            data_bytes = s.recv(buffer)
        except socket.timeout as e:
            err = e.args[0]
            if err == 'timed out':
                time.sleep(1)
                print('Receive time out, retrying...')
                continue
            else:
                print('1___'+str(e))
                os._exit(1)
        except socket.error as e:
            print(('2___'+str(e)))
            os._exit(1)
        else:
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

def imu_callback(host,port):

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try:        
        s.connect((host,port))
        s.settimeout(2)
    except:
        pass

    while True:
        try:
            data_bytes = s.recv(buffer)
        except socket.timeout as e:
            err = e.args[0]
            if err == 'timed out':
                time.sleep(1)
                print('Receive time out, retrying...')
                continue
            else:
                print('1___'+str(e))
                os._exit(1)
        except socket.error as e:
            print(('2___'+str(e)))
            os._exit(1)
        else:
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


if __name__ == '__main__':
    global gps_all
    host = '127.0.0.1'
    port_gps = '5002'
    port_imu = '5004'
    GPSThread = Thread(target=gps_callback, args=(host,port_gps))
    IMUThread = Thread(target=imu_callback, args = (host,port_imu))

    sysID = 1
    gps_ref = GPS(None,-37.62342388464511, 145.12737925483498,0)  # GPS of GMAC

    ##
    ## Initialization
    ##

    time.sleep(10)

    mag0 = imu_all[sysID-1].mag_vector
    acc0 = imu_all[sysID-1].accel
    pos0 = lla2ned(gps_all[sysID-1].lat, gps_all[sysID-1].lon, gps_all[sysID-1].alt)


    node = node(mag0,acc0,pos0)

    settings = settings()
    dt = 0.01 ## The sampling time is based on IMU
    Q_Xsens = True  #If Q_Xsens = False, EKF will solve the Euler angle; 
                #if Q_Xsens = True, EKF will not solve the Euler angle and the angle from sensor output will be used. 
                #In the latter case, the quaternion data should be used
                #For basic experiment, Q_Xsens = False
                #If it is possible, we can run two parallel scripts -- one using Q_Xsens = False and another using Q_Xsens = True
    q_sensor = np.array([]) # variable for quaternion data obtianed from sensor


    C = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) # Matrix transfering ENU to NED, need this to trasnfer IMU data


    try:
        os.makedirs("datalog")
    except FileExistsError:
        pass

    file_name = "datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ekf.csv"


    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['px','py','pz',])
    

        while True:
            gps = gps_all[sysID-1]
            imu = imu_all[sysID-1]

            anchor  = np.array([[1,2],[3,4],[5,6]])
            accel   = imu.accel
            gyros   = imu.gyros
            magVec     = imu.mag_vector

            # Rotate the coordinates 
            accel   = np.dot(C,accel)
            gyros   = np.dot(C,gyros)
            magVec  = np.dot(C.magVec) 

            # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
            IMU = np.concatenate(gyros,accel,magVec)
            ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
            ## The sampling time is based on that of IMU
            time_diff = imu.epoch - gps.epoch
            print("time_diff: ",time_diff)
            if time_diff > 2:   # 2secs after losing the GPS data
                GPS_data = np.array([])
            else:
                GPS_data = np.dot(C,lla2ned( gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt))   # ENU ?

            Dis = np.array([])  # 2D distance !!! Why does it have 3 elements
            
            if Q_Xsens:
                q_sensor = imu.raw_qt
                
            node = EKF(settings,dt,node,IMU,anchor,GPS_data,Dis,Q_Xsens,q_sensor) # EKF

            time.sleep(dt)




