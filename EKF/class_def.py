import numpy as np 
from init_nav_state import *
from init_filter import *

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

#{RAW_QT: 1,2,3,4; MAGNETIC_VECTOR: 1,2,3; ACCELERATION: 1,2,3; EPOCH: 123456789; EULER_321: 1,2,3; MAG_HEADING: 123} 

class IMU:
    def __init__(self, sysID = None, raw_qt = None, mag_vector = None, accel = None, gyros = None, epoch = None, euler = None):
        self.sysID = sysID if sysID is not None else 0
        self.raw_qt = raw_qt if raw_qt is not None else np.zeros([4,1])
        self.mag_vector = mag_vector if mag_vector is not None else np.zeros([3,1])
        self.accel = accel if accel is not None else np.zeros([3,1])
        self.gyros = gyros if gyros is not None else np.zeros([3,1])
        self.epoch = epoch if epoch is not None else 0.0
        self.euler = euler if euler is not None else np.zeros([3,1])

class GPS:
    def __init__(self, sysID = None, lat = None, lon = None, alt = None, epoch = None):
        self.sysID = sysID if sysID is not None else 0
        self.lat = lat if lat is not None else 0.0
        self.lon = lon if lon is not None else 0.0
        self.alt = alt if alt is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0

