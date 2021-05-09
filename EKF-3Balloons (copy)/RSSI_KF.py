import numpy as np
import pandas as pd
import matplotlib.pyplot as plt




class RSSI_Tracker:
    def __init__(self,x):
        self.R = 2
        self.H = np.array([1,0]).reshape(1,2)
        self.Q = 0.005*np.eye(2)
        self.P = np.eye(2)
        self.x = np.array([x,0]).reshape(2,1)
    def update(self,x,P,z,dt):
        xp = np.dot(np.array([[1,dt],[0,1]]),x)
        Pp = P + self.Q
        S = np.dot(np.dot(self.H,Pp),self.H.T) + self.R
        K = np.dot(np.dot(Pp,self.H.T),np.linalg.inv(S))
        info = z - np.dot(self.H,xp)
        corr = np.dot(K,info)
        x = xp + corr
        P = np.dot(np.eye(2) - np.dot(K,self.H),Pp)
        return x,P


if __name__ == "__main__":
    data = pd.read_csv('20210312-160319-RSSILog.txt', header = None)
    RSSI = data.values[:,1]
    Tracker = RSSI_Tracker(RSSI[0])
    x = Tracker.x
    P = Tracker.P
    Filter_RSSI = x[0,0]
    n = 1.7
    A = -60
    dis_RSSI = 10**(-(Filter_RSSI-A)/(n*10))

    for i in  np.arange(1,5485):
        x, P = Tracker.update(x, P, RSSI[i],1)
        Filter_RSSI = np.append(Filter_RSSI,x[0,0])

        dis_RSSI1 = 10**(-(Filter_RSSI[-1]-A)/(n*10))
        dis_RSSI = np.append(dis_RSSI,dis_RSSI1)
    plt.subplot(211)
    plt.plot(RSSI)
    plt.plot(Filter_RSSI)
    plt.subplot(212)
    plt.plot(dis_RSSI)
    plt.show()