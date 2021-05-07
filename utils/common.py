import sys
import numpy as np
sys.path.insert(1,'../utils/')
from navpy import lla2ned
from common_class import *

# def sysID_to_index(sysID):
#     if sysID == 1:
#         return 1
#     elif sysID == 2:
#         return 2
#     elif sysID == 253:
#         return 3
#     elif sysID == 254:
#         return 4
#     elif sysID == 255:
#         return 5
#     else:
#         print('SysID should be in the range 1-5')
#         os._exit(1)
#         return 0

def sysID_to_index(sysID):
    if sysID == 1:
        return 1
    elif sysID == 2:
        return 2
    elif sysID == 3:
        return 3
    elif sysID == 253:
        return 4
    elif sysID == 254:
        return 5
    elif sysID == 255:
        return 6
    else:
        print('SysID should be in the range 1-6')
        os._exit(1)
        return 0

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
    return np.array(array).reshape(len(array),1)

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

def checkRSSI(rssi):
    if rssi.epoch == 0.0:
        return False
    else:
        return True

def checkAllRSSI(rssi_list):
    for i in range(len(rssi_list)):
        if not checkRSSI(rssi_list[i]):
            return False
    return True    

def positionENU(gps,gps_ref):

    C_NED_ENU = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) # correcting acc
    pos_ned = lla2ned(gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt).reshape(3,1)
    pos_enu = np.dot(C_NED_ENU,pos_ned)
    return pos_enu


def list_to_str(list_args):
    list_str = ""
    for i in range(len(list_args)):
        list_str = list_str + str(list_args[i]) + ","
    return list_str[:-1] + "\n"

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
        rssi_i.rssi_filtered = float(extract_string_data("RSSI_filter: ",";",raw_data))
        # rssi_i.distance = float(extract_string_data("distance: ",";",raw_data))
        rssi_i.epoch = float(extract_string_data("time: ",";",raw_data))
        rssi_i.distance = 0
        return True, rssi_i

    except ValueError:

        return False, RSSI()

def extract_str_btw_curly_brackets(data_str):
    string_list = []
    iterator = data_str.find('{')
    
    while data_str.find('}', iterator) != -1:
        substring_end = data_str.find('}', iterator)
        string_list.append(data_str[iterator:substring_end + 1])
        iterator = substring_end + 1
    return string_list

