
from init_filter import *
from init_nav_state import *
from EKF import *
import scipy.io
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.cm as cm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


matplotlib.use("TkAgg")

def quiver_data_to_segments(X, Y, Z, u, v, w):
    segs = np.array((X, Y, Z, u, v, w)).reshape(6,-1)
    new_segs = [[[x,y,z],[u,v,w]] for x,y,z,u,v,w in zip(*segs.tolist())]
    return new_segs

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
    # dt = 0.01
    delta_u_h = np.zeros((6,1))
    gravity = np.array([[0, 0, -9.8184]]).T
    p = np.array([10.0000,    5.0000 ,   0.0175  ,  0.0175  ,  0.3491  ,  0.0200 ,   0.0009])
class node:
    def __init__(self, mag, acc, pos):
        self.x_h,self.angle = init_nav_state(acc, mag, pos)
        self.P,self.Q1,self.Q2 = init_filter(settings)
        self.delta_u_h = np.zeros((9,1))
        self.GPS = pos
        self.dis = np.array([[1, 1, 1,]]).T
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        

mat = scipy.io.loadmat('data.mat')
# acc =  mat['accelBody']
# gyro = mat['gyroBody']

acc = mat['acc'].T
gyro = mat['gyro'].T
GPS = mat['GPS'].T
IMU_t = mat['IMU_t'].T
GPS_t = mat['GPS_t'].T

# acc = mat['acc']
# gyro = mat['gyro']
# mag = mat['mag']
 

# pos = mat['trajPos']

##
## Initialization
##
mag0 = np.array([[10,10,0]]).T
acc0 = np.array([[acc[0,0],acc[0,1], acc[0,2]]]).T
acc0 = np.array([[0,0,-9.81]]).T
pos0 = np.array([[0,0,0]]).T
node = node(mag0,acc0,pos0)
a = np.array([[-0.0],[0.0], [ -0.0],
   [-0.0],
   [ 0.0],
   [-0.0],
      [0.0214],
  [ -0.0539],
   [-0.3399],
    [0.9387]])

node.x_h = a
settings = settings()
anchor = np.array([[1,2],[3,4],[5,6]])

fig = plt.figure(figsize = (5,5),dpi=200)
# ax = fig.add_subplot(111, projection = '3d')

plt.plot(GPS[:,0],GPS[:,1])
plt.pause(.001)

p = plt.plot(node.x_h[0,:],node.x_h[1,:])
for i in np.arange(1,29849):


    dt = 0.01 ## The sampling time is based on IMU
    # IMU data, format: [acc_x,acc_y,axx_z,gyro_x,gyro_y,gyro_z]
    IMU = np.array([[acc[i,0],acc[i,1],acc[i,2],gyro[i,0],gyro[i,1],gyro[i,2]]]).T
    ## GPS and Dis are allowed to be empty, which means that these dara are not available at this sampling time
    
    GPS_data = np.array([])  # should be in Cartesian coordinate.  geodetic_to_geocentric( lat, lon, h)
    Dis = np.array([])  # 2D distance
    t = np.float(i)*dt
    GPS_available = np.where(np.abs(t - GPS_t[0])<0.0001)[0]
    
    if GPS_available.size != 0:
        GPS_data = np.array([[GPS[GPS_available[0],0],GPS[GPS_available[0],1],GPS[GPS_available[0],2]]]).T
        Dis = dis_fun(anchor,GPS_data[0:2].T)
    
    node = EKF(settings,dt,node,IMU,anchor,GPS_data,Dis) # 
    p[0].set_data(node.x_h[0,:],node.x_h[1,:])
    # p[0].set_3d_properties(node.x_h[2,:])
    plt.pause(.001)




# plt.plot(node.x_h[1,:])
# plt.plot(node.x_h[2,:])

plt.show()


if False:

    from mpl_toolkits.mplot3d import Axes3D 
    import matplotlib.pyplot as plt 
    import matplotlib.animation as animation


    def cuboid_data2(o, size=(1,1,1)):
        X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
             [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
            [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
            [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
            [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
            [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
        X = np.array(X).astype(float)
        for i in range(3):
            X[:,:,i] *= size[i]
        X += np.array(o)
        return X

    def plotCubeAt2(positions,sizes=None,colors=None, **kwargs):
        if not isinstance(colors,(list,np.ndarray)): colors=["C0"]*len(positions)
        if not isinstance(sizes,(list,np.ndarray)): sizes=[(1,1,1)]*len(positions)
        g = []
        for p,s,c in zip(positions,sizes,colors):
            g.append( cuboid_data2(p, size=s) )
        return Poly3DCollection(np.concatenate(g),  
                            facecolors=np.repeat(colors,6), **kwargs)
    

# positions = np.array([-0.5,-0.5,-4])
# sizes = [(1,1,1)]
# colors = ["crimson","limegreen"]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')

# pc = plotCubeAt2(positions,sizes,colors=colors, edgecolor="k")
# pc.set_verts([(1,1,1)])

# fig = plt.figure(figsize = (5,5),dpi=200)

# ax = fig.add_subplot(111, projection = '3d')
# ax.axis('equal')
# def update(num,x,y,z,u,plot):
#     plot[0].remove()
#     x_ = x * np.cos(u[num]) - y * np.sin(u[num])
#     y_ = x * np.sin(u[num]) + y * np.cos(u[num])




#     plot[0] = ax.plot_surface(x_, y_, z, cmap="magma")

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = 2 * np.outer(np.cos(u), np.sin(v))
    y = 2 * np.outer(np.sin(u), np.sin(v))
    z = 2 * np.outer(np.ones(np.size(u)), np.cos(v))

    x1 = np.array([0, 2])
    y1 = np.array([0, 0])
    z1 = np.array([0, 0])

    x2 = np.array([0, 0])
    y2 = np.array([0, 2])
    z2 = np.array([0, 0])

    x3 = np.array([0, 0])
    y3 = np.array([0, 0])
    z3 = np.array([0, 2])



    ax.set_xlim(-4,4)
    ax.set_ylim(-4,4)
    ax.set_zlim(-4,4)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    # ax.hlines(y=0, xmin=0, xmax=5, linewidth=2, color='r')
    # plot= [ax.plot_surface(x, y, z,alpha=0,antialiased=False),ax.plot(x1,y1,z1, color='r'),ax.plot(x2,y2,z2, color='r'),ax.plot(x3,y3,z3, color='r')]
    plot= [ax.plot_surface(x,y,z,cmap='bone'),
    ax.plot(x1,y1,z1, color='r'),ax.plot(x2,y2,z2, color='g'),
    ax.plot(x3,y3,z3, color='b'),ax.plot([0, 0],[0, 0],[-2,-3], color='k')]




    while True:
    
        x_ = x * np.cos(i) - y * np.sin(i)
        y_ = x * np.sin(i) + y * np.cos(i)

        plot[0].remove()
        plot[0] = ax.plot_surface(x,y,z,cmap='bone')
    
        # positions[0] = positions[0] * np.cos(i) - positions[1] * np.sin(i)
        # positions[1] = positions[0] * np.cos(i) - positions[1] * np.sin(i)

    
        x1_ = x1 * np.cos(i) - y1 * np.sin(i)
        y1_ = x1 * np.sin(i) + y1 * np.cos(i)
        plot[1][0].set_data(x1_,y1_)
        plot[1][0].set_3d_properties([0,0])
        x2_ = x2 * np.cos(i) - y2 * np.sin(i)
        y2_ = x2 * np.sin(i) + y2 * np.cos(i)
        plot[2][0].set_data(x2_,y2_)
        plot[2][0].set_3d_properties([0,0])
        plt.pause(.001)

        i = i+0.05
plt.show()


