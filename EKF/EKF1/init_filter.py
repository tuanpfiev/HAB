import numpy as np

def init_filter(settings):
    P = np.empty((15, 15, 1))
    P[:,:,0] = np.eye(15)
    
    P[0:3,0:3,0] = np.eye(3)*(settings.p[0]**2)
    P[3:6,3:6,0] = np.eye(3)*(settings.p[1]**2)
    P[6:9,6:9,0] = np.diag(settings.p[2:5]**2)
    P[9:12,9:12,0] = np.eye(3)*(settings.p[5]**2)
    P[12:15,12:15,0] = np.eye(3)*(settings.p[6]**2)

    Q1 = np.zeros((6,6))
    Q1[0:3,0:3] = np.diag(settings.sigma_acc**2)
    Q1[3:6,3:6] = np.diag(settings.sigma_gyro**2)

    Q2 = np.zeros((6,6))
    Q2[0:3,0:3] = np.diag(settings.sigma_acc_bias**2)
    Q2[3:6,3:6] = np.diag(settings.sigma_gyro_bias**2)   

    R = np.eye(15)*settings.sigma_gps**2

    return P, Q1, Q2