
from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 

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
        self.delta_u_h = np.zeros((6,1))
        self.GPS = pos
        self.dis = np.array([[1, 1, 1,]]).T
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

# mat = scipy.io.loadmat('data.mat')


# acc = mat['acc'].T
# gyro = mat['gyro'].T
# GPS = mat['GPS'].T
# IMU_t = mat['IMU_t'].T
# GPS_t = mat['GPS_t'].T


##
## Initialization
##
mag0 = np.array([[10,10,0]]).T
acc0 = np.array([[acc[0,0],acc[0,1], acc[0,2]]]).T
acc0 = np.array([[0,0,-9.81]]).T
pos0 = np.array([[0,0,0]]).T


node = node(mag0,acc0,pos0)

settings = settings()


for i in np.arange(1,29849):
    dt = 0.01 ## The sampling time is based on IMU

    anchor = np.array([[1,2],[3,4],[5,6]])

    # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
    IMU = np.array([[acc[i,0],acc[i,1],acc[i,2],gyro[i,0],gyro[i,1],gyro[i,2]]]).T
    ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
    ## The sampling time is based on that of IMU
    GPS_data = np.array([])  # should be in Cartesian coordinate.  geodetic_to_geocentric( lat, lon, h)
    Dis = np.array([])  # 2D distance

    node = EKF(settings,dt,node,IMU,anchor,GPS_data,Dis) # EKF



