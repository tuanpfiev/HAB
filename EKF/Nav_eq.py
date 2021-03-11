import numpy as np
from utilities import *

def Nav_eq(x,u,dt,g_t):
    q2 = q2dcm(x[6:10])
    u2 = u[0:3]
    f_t = np.dot(q2,u2)
    acc_t = f_t-g_t

    A=np.eye(6)
    A[0,3] = dt
    A[1,4] = dt
    A[2,5] = dt
    B = np.zeros((6,3))
    B[0:3,:] = (dt**2)/2*np.eye(3)

    B[3:6,:] = dt*np.eye(3)

    x[0:6]=np.dot(A,x[0:6])+np.dot(B,acc_t)


    w_tb=u[3:]

    P=w_tb[0]*dt
    Q=w_tb[1]*dt
    R=w_tb[2]*dt


    OMEGA=np.zeros((4,4))
    OMEGA[0,:]=0.5*np.array([0, R, -Q, P])
    OMEGA[1,:]=0.5*np.array([-R, 0, P, Q])
    OMEGA[2,:]=0.5*np.array([Q, -P, 0, R])
    OMEGA[3,:]=0.5*np.array([-P, -Q, -R, 0])

    v=np.linalg.norm(w_tb)*dt

    if v != 0.0:
        M = np.cos(v/2)*np.eye(4)+2/v*np.sin(v/2)*OMEGA
        x[6:] = np.dot(M,x[6:])
    return x
