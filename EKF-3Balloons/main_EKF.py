from init_filter import *
from init_nav_state import *
from EKF_Func import *
import scipy.io
import numpy as np 
from class_def import *
import socket, time, os, sys
from threading import Thread
import GlobalVals
import math
import copy


sys.path.insert(1,'../utils/')
from navpy import lla2ned, ned2lla, lla2ecef
import csv
from common import *
from common_class import *

# def checkRSSI_Calibration():
#     for i in range(len(GlobalVals.RSSI_CALIBRATION_FINISHED)):
#         if not GlobalVals.RSSI_CALIBRATION_FINISHED[i]:
#             return False
#     return True

def checkRSSI_Update(rssi,rssi_prev):
    for i in range(len(rssi)):
        if rssi[i].epoch - rssi_prev[i].epoch == 0 or rssi[i].distance == 0.:
            return False
    return True

    
def enu2lla(enu, gps_ref, latlon_unit='deg', alt_unit='m', model='wgs84'):
    ned = np.dot(GlobalVals.C_ENU_NED,enu)
    lla = ned2lla(ned,gps_ref.lat, gps_ref.lon, gps_ref.alt, latlon_unit, alt_unit, model)
    return lla




def rssi_update(new_data,balloon_id):
    GlobalVals.RSSI[balloon_id] = new_data
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

        while True:
            try:
                data_bytes = s.recv(GlobalVals.GPS_BUFFER)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("gps_callback timed out. Retrying ....")
                    time.sleep(0.1)
                else:
                    print("Exception: " + str(e.__class__))
                    print("There was an error starting the GPS receiver socket. This thread will now stop.")
                    break
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
        string_list = extract_str_btw_curly_brackets(data_str)

        # print("CHECK IMU:",string_list)
        if len(string_list) > 0:
            imu_list = []
            for string in string_list:
                received, imu_i = stringToIMU(string)
                if received:
                    imu_list.append(imu_i)
            
            idx = 0
            with GlobalVals.IMU_UPDATE_MUTEX:
                while idx < len(imu_list):
                    imu_update(imu_list[idx])
                    idx += 1
    s.close()


def distanceRSSI_callback(host,port,balloon_id):

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
                with GlobalVals.BREAK_RSSI_THREAD_MUTEX[balloon_id]:
                    GlobalVals.BREAK_RSSI_THREAD[balloon_id] = True
                return 
        break

    while True:
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX[balloon_id]:
            if GlobalVals.BREAK_RSSI_THREAD[balloon_id]:
                break
        
        while True:
            try:
                data_bytes = s.recv(GlobalVals.RSSI_BUFFER)
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("distanceRSSI_callback timed out. Retrying ...")
                    time.sleep(0.1)
                else:
                    print("Exception: " + str(e.__class__))
                    print("There was an error starting the RSSI receiver socket. This thread will now stop.")
                    break
                break
        
        # print("ID",balloon_id,"Received RSSI: ",data_bytes)

        if len(data_bytes) == 0:
            continue
        
        data_str = data_bytes.decode('utf-8')
        # print('data RSSI: ',data_str)
        string_list = extract_str_btw_curly_brackets(data_str)
        # print(data_str)
        # print("RSSI string:", string_list)
        if len(string_list) > 0:
            rssi_list = []
            for string in string_list:
                received, rssi_i = stringToRSSI(string)
                
                if received:
                    rssi_list.append(rssi_i)
            
            idx = 0
            with GlobalVals.RSSI_UPDATE_MUTEX[balloon_id]:
                while idx < len(rssi_list):
                    # print('updating RSSI')
                    # print(rssi_list[idx].distance)
                    rssi_update(rssi_list[idx],balloon_id)
                    # print(rssi_list[idx].epoch, rssi_list[idx].rssi_filtered)
                    idx += 1
            # print('----------------------------')
    s.close()

# def RSSI_ToDistance(rssi,params):
#     n = params[0][0]
#     A = params[0][1]
    
#     rssi.distance = 10**(-(rssi.rssi_filtered-A)/(n*10))

#     return rssi

# def distanceCalculation(gps1, gps2):
#     p1 = lla2ecef(gps1.lat,gps1.lon,gps1.alt)
#     p2 = lla2ecef(gps2.lat,gps2.lon,gps2.alt)
#     return np.linalg.norm(p1-p2)


# def RSSI_Calibration(rssi,gpsAll,sysID,index):

#     for i in range(len(GlobalVals.REAL_BALLOON)):
#         if GlobalVals.REAL_BALLOON[i] == sysID:
#             neighborRealBalloons = np.delete(GlobalVals.REAL_BALLOON,i,None)
#             break

#     targetBalloon  = neighborRealBalloons[index]
    
#     distance = distanceCalculation(gpsAll[sysID-1],gpsAll[targetBalloon-1])

#     GlobalVals.X[index] = np.concatenate((GlobalVals.X[index],np.array([[rssi.rssi_filtered,1]])),axis=0)
#     GlobalVals.Y[index] = np.concatenate((GlobalVals.Y[index],np.array([[np.log10(distance)]])),axis=0)
    
#     if len(GlobalVals.X[index])< GlobalVals.RSSI_CALIBRATION_SIZE:
#         print("Calibrating RSSI sys ID: ",index,"(",len(GlobalVals.X[index]),"/",GlobalVals.RSSI_CALIBRATION_SIZE,")")
#         return np.ones([1,2]), False, rssi

#     if len(GlobalVals.X[index]) > GlobalVals.RSSI_CALIBRATION_SIZE:
#         GlobalVals.X[index] = np.delete(GlobalVals.X[index],0,0)
#         GlobalVals.Y[index] = np.delete(GlobalVals.Y[index],0,0)

#     # add 2 data point to avoid bifurcation
#     GlobalVals.X[index] = np.concatenate((GlobalVals.X[index],np.array([[-10,1]])),axis=0)
#     GlobalVals.Y[index] = np.concatenate((GlobalVals.Y[index],np.array([[np.log10(1)]])),axis=0)    # distance = 1m, rssi = -1

#     GlobalVals.X[index] = np.concatenate((GlobalVals.X[index],np.array([[-130,1]])),axis=0)
#     GlobalVals.Y[index] = np.concatenate((GlobalVals.Y[index],np.array([[np.log10(20000)]])),axis=0)    # distance = 20000m, rssi = -130

#     w = np.linalg.multi_dot([np.linalg.inv(np.dot(GlobalVals.X[index].transpose(),GlobalVals.X[index])),GlobalVals.X[index].transpose(),GlobalVals.Y[index]])
#     n = -1/(10*w[0][0])
#     A = 10*n*w[1][0]
#     params = np.array([[n,A]])

#     rssi = RSSI_ToDistance(rssi,params)
#     if rssi.distance >10e10:
#         xx = rssi.distance
#     print("Calibrated RSSI sys ID: ",index,", RSSI Distance Error: ", rssi.distance-distance, ", Params (n,A): ",np.array([[n,A]]), )

#     return params, True, rssi



def EKF_Distributor():
    # start socket 
    Distro_Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    Distro_Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
    Distro_Socket.bind((GlobalVals.HOST, GlobalVals.EKF_OUTPUT_DISTRO_SOCKET))
    Distro_Socket.settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT)
    
    # Wait for connection on the distro socket 
    try:
        Distro_Socket.listen(1) 
        Distro_Connection, addr = Distro_Socket.accept()  
        Distro_Connection.settimeout(GlobalVals.EKF_LOGGER_SOCKET_TIMEOUT) 
        print("Logger Connected to ", addr)                                            
    except Exception as e:
        print("Exception: " + str(e.__class__))
        print("Error in the logger socket. Now closing thread.")
        with GlobalVals.BREAK_EKF_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_EKF_DISTRO_THREAD = True
        return

    breakThread = False

    while True:
        if breakThread:
            break

        with GlobalVals.BREAK_EKF_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_EKF_DISTRO_THREAD:
                break

        with GlobalVals.EKF_BUFFER_MUTEX:
            while len(GlobalVals.EKF_BUFFER)>0:
                objEKF = GlobalVals.EKF_BUFFER.pop(0)

                messageStr = "{'system': " + str(objEKF.sysID) + "; 'altitude': " + str(objEKF.alt) + "; 'latitude': " + str(objEKF.lat) + "; 'longitude': " + str(objEKF.lon) + "; 'time': " + str(objEKF.epoch) + "; 'posX': " + str(objEKF.posX) + "; 'posY': " + str(objEKF.posY) +  "; 'p00': " + str(objEKF.p00) +  "; 'p01': " + str(objEKF.p01) + "; 'p10': " + str(objEKF.p10) + "; 'p11': " + str(objEKF.p11) + ";}"
                messageStr_bytes = messageStr.encode('utf-8')

                try:
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

    RSSIThread = [0,0]

    for i in range(GlobalVals.N_REAL_BALLOON-1):
    
        RSSIThread[i] = Thread(target=distanceRSSI_callback, args = (GlobalVals.HOST, GlobalVals.PORT_RSSI[i],i))
        RSSIThread[i].start()

    EKF_DistributorThread = Thread(target = EKF_Distributor, args = ())
    EKF_DistributorThread.start()

    sysID = GlobalVals.SYSID

    print("WAITING for the GPS & RSSI data. Calculation has NOT started yet...")



    while True:
        with GlobalVals.GPS_UPDATE_MUTEX:
            GPS_Status = checkAllGPS(GlobalVals.GPS_ALL)
        print("All GPS ready ?: ", GPS_Status)

        RSSI_Status = checkAllRSSI(GlobalVals.RSSI)
        print("RSSI ready ?:    ", RSSI_Status)

        if GPS_Status and RSSI_Status:
            break
        time.sleep(2)

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
    timeCheck1 = time.time()
    pos0_enu = positionENU(GlobalVals.GPS_ALL[sysID-1], GlobalVals.GPS_REF)
    # print('Time ENU: ',time.time()-timeCheck1)
    node = node(mag0,acc0,pos0_enu)

    settings = settings()
    dt = GlobalVals.LOOPTIME ## The sampling time is based on IMU
    imu_prev_time = 0
    Q_Xsens = True  #If Q_Xsens = False, EKF will solve the Euler angle; 
                #if Q_Xsens = True, EKF will not solve the Euler angle and the angle from sensor output will be used. 
                #In the latter case, the quaternion data should be used
                #For basic experiment, Q_Xsens = False
                #If it is possible, we can run two parallel scripts -- one using Q_Xsens = False and another using Q_Xsens = True
    q_sensor = np.array([]) # variable for quaternion data obtianed from sensor

    C = np.array([[-1,0, 0],[0,-1,0],[0,0,-1]]) # correcting acc

    GPS_data_vel_pre = np.array([])
    GPS_time = 1
    flag = 0

    try:
        os.makedirs("../datalog")
    except FileExistsError:
        pass

    file_name = "../datalog/"+time.strftime("%Y%m%d-%H%M%S")+"-ekf.csv"

    time.sleep(2)

    # logString = "sysID,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,roll,pitch,yaw,gps0_lat,gps0_lon,gps0_alt,gps1_lat,gps1_lon,gps1_alt,gps2_lat,gps2_lon,gps2_alt,gps3_lat,gps3_lon,gps3_alt,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,qt1,qt2,qt3,qt4,epoch,magVec1,magVec2,magVec3,rssi_distance 1,rssi_distance 2,node_P\n"
    with open(file_name,'w') as file:
        output = csv.writer(file)
        output.writerow(['sysID','x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','roll','pitch','yaw',\
            'gps0_lat','gps0_lon','gps0_alt','gps1_lat','gps1_lon','gps1_alt','gps2_lat','gps2_lon','gps2_alt','gps3_lat','gps3_lon','gps3_alt','gps4_lat','gps4_lon','gps4_alt',\
                'gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z','qt1','qt2','qt3','qt4','epoch',\
                    'magVec1','magVec2','magVec3','rssi_distance 1','rssi_distance 2','anchor_distance0','anchor_distance1','anchor_distance2'])
        # try:
        #     fileObj = open(file_name, "a")
        #     fileObj.write(logString)
        #     fileObj.close()
        # except Exception as e:
        #     print("EKF: Error writting to file. Breaking thread.")
        #     print("EKF: Exception: " + str(e.__class__))
        # gps_all_prev = GlobalVals.GPS_ALL
        gps_prev = GPS()
        imu_prev = IMU()
        rssi_prev = np.array([RSSI()]*(GlobalVals.N_REAL_BALLOON-1))
        epoch_prev = 0
        flag_start = True
        rssi = [None]*(GlobalVals.N_REAL_BALLOON-1)

        flagRSSI_Calibration = True

        while True:
            timeLoopStart = time.time()
            # print('1')
            with GlobalVals.GPS_UPDATE_MUTEX:
                gps_all = copy.deepcopy(GlobalVals.GPS_ALL)
                # print('1')
            gps = gps_all[sysID-1]
            # print('2')
            with GlobalVals.IMU_UPDATE_MUTEX:
                imu = copy.deepcopy(GlobalVals.IMU_ALL[sysID-1])
                # print('2')
            # print('3')
            timeCheck3 = time.time()
            for i in range(GlobalVals.N_REAL_BALLOON-1):
                with GlobalVals.RSSI_UPDATE_MUTEX[i]:
                    rssi[i] = copy.deepcopy(GlobalVals.RSSI[i])
                    # print('3')
                # if rssi[i].epoch != rssi_prev[i].epoch:
                #     GlobalVals.RSSI_PARAMS[i], GlobalVals.RSSI_CALIBRATION_FINISHED[i],rssi[i] = RSSI_Calibration(rssi[i],gps_all,sysID,i)
                    # rssi[i] = RSSI_ToDistance(rssi[i],GlobalVals.RSSI_PARAMS[i])
                    # dis = distanceCalculation(gps_all[sysID-1],gps_all[i])
                    # print("RSSI Distance Err :", rssi[i].distance-dis)
            
            # print('Time RSSI Calib: ',time.time()-timeCheck3)

            # if not checkRSSI_Calibration():
            #     rssi_prev = copy.deepcopy(rssi)
            #     continue
            
            if flag_start:
                dt = GlobalVals.LOOPTIME
            else:
                dt = (imu.epoch - imu_prev.epoch)/1000

            if dt > 0 or flag_start:
                
                # print('dt: ',dt)
                epoch =time.time()
                # epoch = imu.epoch

                timeLocal = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch))
                anchor_position = np.zeros([len(GlobalVals.ANCHOR),2])
                if checkAllGPS(gps_all):
                    for i in range(len(GlobalVals.ANCHOR)):
                        posEN = positionENU(gps_all[GlobalVals.ANCHOR[i]-1],gps_ref).T
                        anchor_position[i,:]=posEN[0][0:2]   
                
                # Rotate the coordinates 
                accel   = np.dot(C,imu.accel)

                # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
                IMU_i = np.concatenate((accel,imu.gyros,imu.mag_vector))
                ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
                ## The sampling time is based on that of IMU
                # timeGPS_IMU_diff = imu.epoch/1000 - gps.epoch       # IMU epoch is in ms
                # timeRSSI_IMU_diff = imu.epoch/1000 - rssi[0].epoch
                # timeRSSI_IMU_diff = imu.epoch/1000 - rssi[0].epoch

                # print("time_GPS_diff: ",timeGPS_IMU_diff)
                # print("time_RSSI_diff: ",timeRSSI_IMU_diff)
                
                GPS_data = np.array([])
                GPS_data_vel = np.array([])  # Should be in Cartesian coordinate.  geodetic_to_geocentric( lat, lon, h)
                # print(rssi.epoch)
                if checkGPS(gps) and gps.epoch - gps_prev.epoch> 0:
                    GPS_data = positionENU(gps,gps_ref)
                    # GPS_data[2] = -5460.8                           ########## WHYYYYYY
                    gps_prev = copy.deepcopy(gps)
                    # print('update GPS')
                    # print(GPS_data)

                if GPS_data.size != 0:
                    if flag == 0:
                        v = np.zeros((3,1))
                        # print(GPS_data)
                        GPS_data_vel = np.concatenate((GPS_data,v))  
                        GPS_data_vel_pre = GPS_data_vel
                        GPS_time_pre = gps.epoch
                        flag = 1
                    else:                    
                        dt_GPS = gps.epoch - GPS_time_pre
                        v[0] = (GPS_data[0]-GPS_data_vel_pre[0])/dt_GPS
                        v[1] = (GPS_data[1]-GPS_data_vel_pre[1])/dt_GPS
                        v[2] = (GPS_data[2]-GPS_data_vel_pre[2])/dt_GPS
                        GPS_data_vel = np.concatenate((GPS_data,v))  
                        GPS_data_vel = np.concatenate((GPS_data,v))  
                        GPS_data_vel_pre = GPS_data_vel
                        GPS_time_pre = gps.epoch
                        # print(dt_GPS)

                anchor_distance = np.array([])
                # print("RSSI update: ",checkRSSI_Update(rssi,rssi_prev))
                # print(rssi[0].distance)
                if checkAllGPS(gps_all) and checkRSSI_Update(rssi,rssi_prev):
                    # print('update rssi')
                    anchor_distance = np.zeros([len(GlobalVals.ANCHOR),1])           
                    for i in range(len(GlobalVals.ANCHOR)):
                        if GlobalVals.ANCHOR[i] not in GlobalVals.REAL_BALLOON:
                            temp = distance2D([gps, gps_all[i-1],GlobalVals.GPS_REF])
                            anchor_distance[i,:] = distance2D([gps, gps_all[GlobalVals.ANCHOR[i]-1],GlobalVals.GPS_REF])
                            
                        else:
                            temp = distance2D([gps, gps_all[i-1],GlobalVals.GPS_REF,rssi[i].distance])
                            anchor_distance[i,:] = distance2D([gps, gps_all[GlobalVals.ANCHOR[i]-1],GlobalVals.GPS_REF,rssi[i].distance])
                    
                    rssi_prev = copy.deepcopy(rssi)

                    # print(anchor_distance)
                    # print("===============")

                if Q_Xsens:
                    q_sensor = imu.raw_qt.reshape(4)
                    q_sensor = np.roll(q_sensor,-1)

                timeCheck2 = time.time()
                node = EKF_Func(settings,dt,node,IMU_i,anchor_position,GPS_data_vel,anchor_distance,Q_Xsens,q_sensor) # EKF
                # print('EKF Time: ',time.time()-timeCheck2)
                x_h = np.array([node.x_h[:,-1]]).T


                # just for plotting
                if anchor_distance.size == 0:
                    anchor_distance = -np.ones([len(GlobalVals.ANCHOR),1])           
                output.writerow([GlobalVals.SYSID, x_h[0][0],x_h[1][0],x_h[2][0],x_h[3][0],x_h[4][0],x_h[5][0],x_h[6][0],x_h[7][0],x_h[8][0],x_h[9][0],node.roll,node.pitch,node.yaw,\
                    gps_all[0].lat, gps_all[0].lon, gps_all[0].alt, gps_all[1].lat, gps_all[1].lon, gps_all[1].alt, gps_all[2].lat, gps_all[2].lon, gps_all[2].alt, gps_all[3].lat, gps_all[3].lon, gps_all[3].alt, gps_all[4].lat, gps_all[4].lon, gps_all[4].alt,
                        imu.gyros[0][0],imu.gyros[1][0],imu.gyros[2][0],accel[0][0],accel[1][0],accel[2][0],imu.raw_qt[0][0],imu.raw_qt[1][0],imu.raw_qt[2][0],imu.raw_qt[3][0],epoch,\
                            imu.mag_vector[0][0],imu.mag_vector[1][0],imu.mag_vector[2][0],rssi[0].distance,rssi[1].distance, anchor_distance[0][0], anchor_distance[1][0], anchor_distance[2][0]])


                posENU_EKF = np.array([x_h[0][0],x_h[1][0],x_h[2][0]]).T
                objEKF = enu2lla(posENU_EKF, gps_ref)
                print('Lon: ',round(objEKF[1],2), ', Lat: ', round(objEKF[0],2), ', Alt: ', round(objEKF[2],1), 'Time: ',timeLocal)

                with GlobalVals.EKF_BUFFER_MUTEX:
                    GlobalVals.EKF_BUFFER.append(EKF(sysID, objEKF[0],objEKF[1],objEKF[2],epoch,node.x_h[0][0],x_h[1][0],node.P[0][0],node.P[0][1],node.P[1][0],node.P[1][1]))

                # gps_all_prev = copy.deepcopy(gps_all)
                
                imu_prev = copy.deepcopy(imu)
                epoch_prev = copy.deepcopy(epoch)
                flag_start = False
            
            # Sleep
            elapsed = time.time()-timeLoopStart
            
            if GlobalVals.LOOPTIME - elapsed > 0:
                print('Elapsed Time: ',elapsed, "remaining time: ",GlobalVals.LOOPTIME - elapsed  )
                time.sleep(GlobalVals.LOOPTIME - elapsed)



    if GPSThread.is_alive():
        with GlobalVals.BREAK_GPS_THREAD_MUTEX:
            GlobalVals.BREAK_GPS_THREAD = True
        GPSThread.join()

    if IMUThread.is_alive():
        with GlobalVals.BREAK_IMU_THREAD_MUTEX:
            GlobalVals.BREAK_IMU_THREAD = True
        IMUThread.join()

    # if RSSIThread_0.is_alive():
    #     with GlobalVals.BREAK_RSSI_THREAD_MUTEX[0]:
    #         GlobalVals.BREAK_RSSI_THREAD[0] = True
    #     RSSIThread_0.join()

    for i in range(GlobalVals.N_REAL_BALLOON-1):
        with GlobalVals.BREAK_RSSI_THREAD_MUTEX[i]:
            GlobalVals.BREAK_RSSI_THREAD[i] = True
        RSSIThread[i].join()

    if EKF_DistributorThread.is_alive():
        with GlobalVals.BREAK_EKF_DISTRO_THREAD_MUTEX:
            GlobalVals.BREAK_EKF_DISTRO_THREAD = True
        EKF_DistributorThread.join()


