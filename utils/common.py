import numpy as np
def sysID_to_index(sysID):
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

def positionENU(gps,gps_ref):
    pos_ned = lla2ned(gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt).reshape(3,1)
    pos_enu = np.dot(GlobalVals.C_NED_ENU,pos_ned)
    return pos_enu

