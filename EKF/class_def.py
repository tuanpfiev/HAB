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
        self.delta_u_h = np.zeros((9,1))
        self.GPS = pos
        self.dis = np.array([[1, 1, 1,]]).T
        self.roll = np.array([])
        self.pitch = np.array([])
        self.yaw = np.array([])
        self.x_apo = np.array([])
        self.P_apo = (10*np.ones((12,12)))


