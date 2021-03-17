import numpy as np 

def Rt2b(ang):
    cr = np.cos(ang[0])
    sr = np.sin(ang[0])

    cp = np.cos(ang[1])
    sp = np.sin(ang[1])

    cy = np.cos(ang[2])
    sy = np.sin(ang[2])

    R = np.array([[cy*cp, sy*cp, -sp],
        [-sy*cr+cy*sp*sr, cy*cr+sy*sp*sr, cp*sr],
        [sy*sr+cy*sp*cr, -cy*sr+sy*sp*cr, cp*cr] ])

    return R


def decm2q(R):
    q = np.zeros((4,1))
    q[3] = 0.5*np.sqrt( 1 + sum(np.diag(R)))
    q[0] = (R[2,1] - R[1,2])/q[3]*0.25
    q[1] = (R[0,2] - R[2,0])/q[3]*0.25
    q[2] = (R[1,0] - R[0,1])/q[3]*0.25
    return q

def get_Rb2p():
    return np.array([[0.9880,   -0.1472,   -0.0463],
        [0.1540 ,   0.9605,    0.2319],
        [0.0103,   -0.2363,    0.9716]])


def q2dcm(q):
    p = np.zeros((6,1))

    p[0:4] = (q**2)

    p[4] = p[1] + p[2]

    if p[0]+p[3]+p[4] is not 0:
       p[5]=2/(p[0]+p[3]+p[4]); 
    else:
        p[5]=0

    R = np.zeros((3,3))
    R[0,0]=1-p[5]*p[4]
    R[1,1]=1-p[5]*(p[0]+p[2])
    R[2,2]=1-p[5]*(p[0]+p[1])

    p[0] = p[5]*q[0]
    p[1] = p[5]*q[1]
    p[4] = p[5]*q[2]*q[3]
    p[5] = p[0]*q[1]

    R[0,1] = p[5]-p[4]
    R[1,0] =p[5]+p[4]

    p[4] = p[1]*q[3]
    p[5] = p[0]*q[2]

    R[0,2] = p[5]+p[4]
    R[2,0] = p[5]-p[4]

    p[4] = p[0]*q[3]
    p[5] = p[1]*q[2]

    R[1,2] = p[5]-p[4]
    R[2,1] = p[5]+p[4]
    return R

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))
    
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def dcm2euler(U):
    y = np.zeros((3,1))
    y[0]= np.arctan2(U[2,1],U[2,2])
    y[1] = np.arcsin(-U[2,0]) 
    y[2] = -np.arctan(U[1,0]/U[0,0])
    return y

def dis_fun(anchor,node_pos):
    dis = np.zeros((anchor.shape[0],1))
    for i in range(anchor.shape[0]): 
        dis[i] = np.linalg.norm((anchor[i,:]-node_pos),2)
    return dis

from math import cos, radians, sin, sqrt

# Ellipsoid parameters: semi major axis in metres, reciprocal flattening.


def geodetic_to_geocentric(latitude, longitude, height):
    GRS80 = 6378137, 298.257222100882711
    WGS84 = 6378137, 298.257223563
    phi = radians(latitude)
    lambda_angle = radians(longitude)
    sin_phi = sin(phi)
    a, rf = WGS84              # semi-major axis, reciprocal flattening
    e2 = 1 - (1 - 1 / rf) ** 2  # eccentricity squared
    n = a / sqrt(1 - e2 * sin_phi ** 2) # prime vertical radius
    r = (n + height) * cos(phi)   # perpendicular distance from z axis
    x = r * cos(lambda_angle)
    y = r * sin(lambda_angle)
    z = (n * (1 - e2) + height) * sin_phi
    return x, y, z
