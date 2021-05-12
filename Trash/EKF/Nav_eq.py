import numpy as np
from utilities import *

def Nav_eq(x,u,dt,g_t,x_apo,P_apo,R_data,Q_data):
    q2 = q2dcm(x[6:10])
    u2 = u.copy()
    u[0:3] = u2[3:6]
    u[3:6] = u2[0:3]
    u2 = u[3:6]
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


    u[6:9] = u[6:9]/np.linalg.norm(u[6:9],2)


    # # previous code##
    # w_tb=u[3:]

    # P=w_tb[0]*dt
    # Q=w_tb[1]*dt
    # R=w_tb[2]*dt


    # OMEGA=np.zeros((4,4))
    # OMEGA[0,:]=0.5*np.array([0, R, -Q, P])
    # OMEGA[1,:]=0.5*np.array([-R, 0, P, Q])
    # OMEGA[2,:]=0.5*np.array([Q, -P, 0, R])
    # OMEGA[3,:]=0.5*np.array([-P, -Q, -R, 0])

    # v=np.linalg.norm(w_tb)*dt

    # if v != 0.0:
    #     M = np.cos(v/2)*np.eye(4)+2/v*np.sin(v/2)*OMEGA
    #     x[6:] = np.dot(M,x[6:])
    # #

    if x_apo.size == 0:
        gyro_init = np.array([[0],[0],[0]])
        gyro_acc_init = np.array([[0],[0],[0]])
        acc_init = np.array([[0],[0],[-9.822]])
        mag_init=np.array(u[6:9]).reshape(3,1)
        x_apo = np.concatenate((gyro_init,gyro_acc_init,acc_init,mag_init)) #np.array([gyro_init,gyro_acc_init,acc_init,mag_init])

    wx = x_apo[0,0]
    wy = x_apo[1,0]
    wz = x_apo[2,0]

    wax = x_apo[3,0]
    way = x_apo[4,0]
    waz = x_apo[5,0]

    zex = x_apo[6,0]
    zey = x_apo[7,0]
    zez = x_apo[8,0]

    mux = x_apo[9,0]
    muy = x_apo[10,0]
    muz = x_apo[11,0]

    wak = np.array([[wax],[way],[waz]])
    wk = np.array([[wx],[wy],[wz]]) + dt*wak
    O = np.array([[0,-wz,wy],
        [wz,0,-wx],
        [-wy,wx,0]]).T
    O2 = np.dot(O,O)
    z_v = np.array([[zex],[zey],[zez]])
    m_v = np.array([[mux],[muy],[muz]])
    zek = np.dot((np.eye(3)+O*dt+dt**2/2*O2),z_v)
    muk =  np.dot((np.eye(3)+O*dt+dt**2/2*O2),m_v)
    
    x_apr=np.concatenate((wk,wak,zek,muk))

    EZ = np.array([[0,zez,-zey],
                    [-zez,0,zex],
                    [zey,-zex,0]]).T

    MA = np.array([[0,muz,-muy],
            [-muz,0,mux],
            [muy,-mux,0]]).T
    E = np.eye(3)
    Z = np.zeros((3,3))

    A_lin=np.block([[Z,  E,  Z,  Z],
        [Z,Z,Z,Z],
    [EZ,Z,O,Z],
    [MA,Z,Z,O]])
    A_lin=np.eye(12) + A_lin * dt

    q_rotSpeed = Q_data[0]
    q_rotAcc = Q_data[1]
    q_acc = Q_data[2]
    q_mag = Q_data[3]
    Q = np.diag([q_rotSpeed,q_rotSpeed,q_rotSpeed,
    q_rotAcc,q_rotAcc,q_rotAcc,
    q_acc,q_acc,q_acc,
    q_mag,q_mag,q_mag])

    P_apr = np.dot(np.dot(A_lin,P_apo),A_lin.T)+Q


    r_gyro = R_data[0]
    r_accel = R_data[1]
    r_mag = R_data[2]
    R = np.diag([r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel,r_mag,r_mag,r_mag])

    H_k = np.block([[  E,     Z,      Z,    Z],
        [Z,     Z,      E,    Z],
        [Z,     Z,      Z,    E]])
    y_k1 = np.dot(H_k,x_apr)
    y_k = u[0:9] - y_k1

    S_k = np.dot(np.dot(H_k,P_apr),H_k.T)
    S_k_inv = np.linalg.inv(S_k + R)
    K_k = np.dot(np.dot(P_apr,H_k.T),S_k_inv)
    x_apo = x_apr+np.dot(K_k,y_k)
    P_apo=np.dot((np.eye(12)-np.dot(K_k,H_k)),P_apr)
    norm1 = np.linalg.norm(x_apo[6:9],2) 
    z_n_b = -x_apo[6:9]/norm1
    norm2 = np.linalg.norm(x_apo[9:12],2)
    m_n_b = x_apo[9:12]/norm2

    y_n_b = np.cross(z_n_b.T,m_n_b.T)
    y_n_b=y_n_b/np.linalg.norm(y_n_b,2)

    x_n_b = np.cross(y_n_b,z_n_b.T)
    x_n_b = x_n_b/np.linalg.norm(x_n_b,2)

    Rot_matrix = np.hstack((x_n_b.T,y_n_b.T,z_n_b))
    # angle = np.rad2deg(rotationMatrixToEulerAngles(Rot_matrix))
    # yaw = np.arctan2(Rot_matrix[1,2],Rot_matrix[2,2])
    # pitch  = -np.arcsin(Rot_matrix[0,2])
    # roll = np.arctan2(Rot_matrix[0,1],Rot_matrix[0,0])
    q = decm2q(Rot_matrix)
    a = np.rad2deg(rotationMatrixToEulerAngles(Rot_matrix))
    # b = np.rad2deg([yaw,pitch,roll])
    x[6:10] = q
    return x, x_apo, P_apo, a
