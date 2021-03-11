import numpy as np 
from utilities import *

# x y z vx vy vz q0 q1 q2 q3 
def state_model(x,u,Ts):
    q2 = q2dcm(x[6:10])
    u2 = u[0:3]
    f_t = np.dot(q2,u2)

    St = np.array([[0, -f_t[2], f_t[1]],
                    [f_t[2], 0, -f_t[0]],    
                    [-f_t[1],f_t[0],0]])
    Fc = np.zeros((15,15))
    Fc[0:3,3:6] = np.eye(3)
    Fc[3:6,6:9] = St
    Fc[3:6,9:12] = q2
    Fc[6:9,12:15] = -q2
    F = np.eye(15) + Ts*Fc
    G = np.zeros((15,12))
    G[3:6,0:3] = q2
    G[6:9,3:6] = -q2
    G[9:15,6:12] = np.eye(6)
    G = Ts*G
    return F,G


