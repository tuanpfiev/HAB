from Nav_eq import *
from state_model import *
from utilities import *
def EKF(settings,dt,node,IMU,anchor,GPS,Dis,Q_Xsens,q_sensor):
    # anchor = anchor.T
    Ts = dt
    u = IMU
    
    # node.x_h = np.column_stack((node.x_h[:-1], node.x_h))
    x_h = np.array([node.x_h[:,-1]]).T
    P = node.P[:,:,-1]
    Q1 = node.Q1
    Q2 = node.Q2
    delta_u_h = node.delta_u_h
    u_h = u + delta_u_h

    if Q_Xsens == True:
        # x_h[6:,-1] = q_sensor
        x_h[6:,-1] = q_sensor
        q2 = q2dcm(x_h[6:10])
        u2 = u[0:3]
        f_t = np.dot(q2,u2)
        acc_t = f_t-settings.gravity
        
        A=np.eye(6)
        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt
        B = np.zeros((6,3))
        B[0:3,:] = (dt**2)/2*np.eye(3)
        
        B[3:6,:] = dt*np.eye(3)
        
        x_h[0:6]=np.dot(A,x_h[0:6])+np.dot(B,acc_t)
    else:
        R_data = np.array([1e-4,1e-2,1e-3])
        Q_data = np.array([1e-5,1e-5,1e-7,1e-7])
        
        x_h, node.x_apo, node.P_apo, a = Nav_eq(x_h,u_h,Ts,settings.gravity,node.x_apo,node.P_apo,R_data,Q_data)
        [node.yaw, node.pitch, node.roll] = a
        
        node.angle = np.column_stack((node.angle,a)) 

    F, G = state_model(x_h,u_h,Ts)
    Q = np.eye(Q1.shape[0]+Q2.shape[0])
    Q[0:6,0:6] = Q1
    Q[6:12,6:12] = Q2
    P = np.dot(np.dot(F,P),F.T)+np.dot(np.dot(G,Q),G.T)
    # dcm = q2dcm(np.array([x_h[6:,-1]]).T)
    # a = np.rad2deg(rotationMatrixToEulerAngles(q2dcm(np.array([x_h[6:,-1]]).T)))
    # a = np.array([a]).T

     

    Rn2p = get_Rb2p()*q2dcm(x_h[-4:]).T
    H = np.zeros((9,15))
    H[0:3,0:3] = np.eye(3)
    H[3:6,3:6] = Rn2p
    dis = np.zeros((3,))
    for i in np.arange(3):
        dis[i] = np.sqrt(np.sum((x_h[0:2].T  - anchor[i,0:2])**2))
        H[6+i,0] = (x_h[0,-1]-anchor[i,0])/dis[i]
        H[6+i,1] = (x_h[1,-1]-anchor[i,1])/dis[i]
    R  = np.zeros((9,9))
    R[0:3,0:3] = settings.sigma_gps**2*np.eye(3)
    R[3,3] = 1
    R[4:6,4:6] = 1*np.eye(2)
    R[6:9,6:9] = settings.sigma_dis**2*np.eye(3)

    ind = np.zeros((9,),dtype=bool)
    y = np.zeros((9,1))
    y[3:6] = np.zeros((3,1))

    if GPS.size != 0:
        y[0:3] = GPS
        ind[0:3] = ~ind[0:3]
    if Dis.size != 0:
        y[6:9] = Dis
        ind[6:9] = ~ind[6:9]

    # y_new = y
    # H_new = H    
    # R_new = R
    yhat = np.zeros((9,1))
    yhat = np.dot(H[:,0:6],x_h[0:6])        
    yhat[6:9] = np.array([dis]).T
    # yhat_new = yhat
    y = np.delete(y,np.where(ind==False),0)
    yhat = np.delete(yhat,np.where(ind==False),0)
    H = np.delete(H,np.where(ind==False),0)
    R = np.delete(R,np.where(ind==False),0)
    R = np.delete(R,np.where(ind==False),1)
    K= np.dot(np.dot(P,H.T),np.linalg.inv(np.dot(np.dot(H,P),H.T)+R))
    z = np.zeros((15,1))
    z[9:] = delta_u_h[0:6]
    z = z + np.dot(K,y-yhat)
    x_h[0:6] = x_h[0:6] + z[0:6]
    x_h[6:10] = Gamma(x_h[6:10],z[6:9])
    delta_u_h=z[9:15]
    P = np.dot(np.eye(15)-np.dot(K,H),P)
    node.x_h = np.column_stack((node.x_h,x_h))
    node.delta_u_h[0:6] = delta_u_h
    node.P = np.dstack((node.P,P))
    return node


def Gamma(q,ep):
    R = q2dcm(q)
    Omega = np.array([[0,-ep[2],ep[1]],
                    [ep[2],0,-ep[0]],
                    [-ep[1], ep[0], 0]])
    R1 = np.dot(np.eye(3) - Omega,R)
    return decm2q(R1)


