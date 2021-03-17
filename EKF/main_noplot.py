from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 
from class_def import *
import socket, time, os
from threading import Thread


sys.path.insert(1,'../utils')
from navpy import lla2ecef
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

    ##
    ## Initialization
    ##

    time.sleep(10)

    mag0 = imu_all[sysID-1].mag_vector.T
    acc0 = imu_all[sysID-1].accel.T
    gps0 = np.array([gps_all[sysID-1].lat, gps_all[sysID-1].lon, gps_all[sysID-1].alt])
    pos0 = lla2ecef(gps0)

    # mag0 = np.array([[10,10,0]]).T
    # acc0 = np.array([[acc[0,0],acc[0,1], acc[0,2]]]).T
    # acc0 = np.array([[0,0,-9.81]]).T
    # pos0 = np.array([[0,0,0]]).T

    node = node(mag0,acc0,pos0)

    settings = settings()
    dt = 0.01 ## The sampling time is based on IMU

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

            anchor = np.array([[1,2],[3,4],[5,6]])
            accel = imu.accel
            gyros = imu.gyros

            # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
            IMU = np.array([accel[0],accel[1],accel[2],gyros[0],gyros[1],gyros[2]).T
            ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
            ## The sampling time is based on that of IMU
            time_diff = imu.epoch - gps.epoch
            print("time_diff: ",time_diff)
            if time_diff > 2:   # 2secs after losing the GPS data
                GPS_data = np.array([])
            else:
                GPS_data = lla2ecef(gps.lat, gps.lon, gps.alt)

            GPS_data = np.array([])  # should be in Cartesian coordinate.  geodetic_to_geocentric( lat, lon, h)
            Dis = np.array([])  # 2D distance !!! Why does it have 3 elements

            node = EKF(settings,dt,node,IMU,anchor,GPS_data,Dis) # EKF

            time.sleep(dt)




