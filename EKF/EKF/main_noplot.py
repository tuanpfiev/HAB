from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
# import time
# import pyprind

class settings:
    ##
    ## Set EKF parameters 
    ##
    sigma_acc = np.array([0.05, 0.05, 0.05])
    sigma_gyro = np.array([0.1*np.pi/180, 0.1*np.pi/180, 0.1*np.pi/180])
    sigma_acc_bias = np.array([1e-4, 1e-4, 1e-4])
    sigma_gyro_bias = np.array([0.01*np.pi/180, 0.01*np.pi/180, 0.01*np.pi/180])
    sigma_dis= np.array([0.1, 0.1, 0.1])
    sigma_gps = .3/np.sqrt(3)
    delta_u_h = np.zeros((6,1))
    gravity = np.array([[0, 0, -9.8184]]).T
    p = np.array([10.0000,    5.0000 ,   0.0175  ,  0.0175  ,  0.3491  ,  0.0200 ,   0.0009])
class node:
    def __init__(self, mag, acc, pos):
        self.x_h,self.angle = init_nav_state(acc, mag, pos) #initial setup goes here
        self.P,self.Q1,self.Q2 = init_filter(settings)
        self.delta_u_h = np.zeros((9,1))
        self.GPS = pos
        self.dis = np.array([[1, 1, 1,]]).T
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.x_apo = np.array([])
        self.P_apo = (10*np.ones((12,12)))

########### test code -- please ignore #######
# mat = scipy.io.loadmat('data.mat')


# acc = mat['acc'].T
# gyro = mat['gyro'].T
# GPS = mat['GPS'].T
# IMU_t = mat['IMU_t'].T
# GPS_t = mat['GPS_t'].T
##################################

##
## Initialization
##
mag0 = np.array([[10,10,0]]).T
acc0 = np.array([[-0.0008,0.0012, -0.0062]]).T
acc0 = np.array([[0,0,-9.81]]).T
pos0 = np.array([[0,0,0]]).T


node = node(mag0,acc0,pos0)

settings = settings()

# ########### test code --- please ignore ########################
# data = pd.read_csv('data_xsens3.txt',
#                            sep='\t',
#                            skiprows=4, 
#                            index_col=False)
    
#         # Extract the columns that you want, and pass them on
# in_data = { 'acc_x':   data.filter(regex='r_acc_x').values,
#             'acc_y':   data.filter(regex='r_acc_y').values,
#             'acc_z':   data.filter(regex='r_acc_z').values,
#             'omega_x':   data.filter(regex='gyr_x').values,
#             'omega_y':   data.filter(regex='gyr_y').values,
#             'omega_z':   data.filter(regex='gyr_z').values,
#             'mag_x':   data.filter(regex='mag_x').values,
#             'mag_y':   data.filter(regex='mag_y').values,
#             'mag_z':   data.filter(regex='mag_z').values,
#                'q_w':data.filter(regex='q_w').values,
#                'q_i':data.filter(regex='q_i').values,
#                'q_j':data.filter(regex='q_j').values,
#                'q_k':data.filter(regex='q_k').values}
# acc_x = in_data['acc_x']
# acc_y = in_data['acc_y']
# acc_z = in_data['acc_z']

# omega_x = in_data['omega_x']
# omega_y = in_data['omega_y']
# omega_z = in_data['omega_z']

# mag_x = in_data['mag_x']
# mag_y = in_data['mag_y']
# mag_z = in_data['mag_z']
# ##################################

Q_Xsens = True  #If Q_Xsens = False, EKF will solve the Euler angle; 
                #if Q_Xsens = True, EKF will not solve the Euler angle and the angle from sensor output will be used. 
                #In the latter case, the quaternion data should be used
                #For basic experiment, Q_Xsens = False
                #If it is possible, we can run two parallel scripts -- one using Q_Xsens = False and another using Q_Xsens = True
q_sensor = np.array([]) # variable for quaternion data obtianed from sensor


C = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) # Matrix transfering ENU to NED, need this to trasnfer IMU data
while True:
    dt = 0.1  ## The sampling time is based on IMU, unit second. Can be fixed or update using timestamp

    anchor = np.array([[1,2],[3,4],[5,6]]) # anchor position, needs update real-time

    #################### IMU data ####################
    acc_1 = np.array([-0.172541,0.145353,9.82087]).reshape(3,1)             # acc data 3X1 vector
    omega_1 = np.array([ 0.00115498,-0.000840643,0.00624861 ]).reshape(3,1)               # gyro data 3X1 vector
    mag_1 = np.array([-0.323004,0.078397,0.972453]).reshape(3,1)                  # mag data 3X1 vector

    acc_1 = np.dot(C,acc_1)
    omega_1 = np.dot(C,omega_1)
    mag_1 = np.dot(C,mag_1)

    IMU = np.concatenate((omega_1,acc_1,mag_1)) # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]'


    ################################################
    ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
    ## The sampling time is based on that of IMU
    GPS_data = np.array([])  # Should be in Cartesian coordinate.  geodetic_to_geocentric( lat, lon, h)
    Dis = np.array([])       # 2D distance; If 3D distance is used, the altitude can be used to get the 2D distance, otherwise one additional anchor should be used

    ################## quaternion data from sensor#################
    if Q_Xsens == True:
        q_i = 0.0095148
        q_j = -0.0079009
        q_k = -0.99645
        q_w = -0.083255
        q_sensor = np.array([q_i,q_j,q_k,q_w]).reshape(4)

    node = EKF(settings,dt,node,IMU,anchor,GPS_data,Dis,Q_Xsens,q_sensor) # EKF
    ##### Please save variable 'node' #########



