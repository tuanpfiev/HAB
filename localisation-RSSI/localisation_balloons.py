import numpy as np
import concurrent.futures
from scipy.stats import hypergeom
from scipy.special import hyp1f1
from scipy.io import loadmat
from scipy.optimize import fmin,fmin_powell,fmin_cg,minimize
import inspect
from scipy import random
from  Ambi_Resolve_A import Ambi_resolve
import GlobalVals

import sys
sys.path.insert(1,'../utils/')
from navpy import lla2ned, ned2lla, lla2ecef
import csv
from common import *
from common_class import *


def KLD1(x,mu_r,sigma_r):
    # x[2] = 1
    return 1/2*1/sigma_r*(np.linalg.norm(np.array([x[0]-mu_r[0],x[1]-mu_r[1]]))**2+2*0.5**2)-np.log(x[2]**2/sigma_r)

def KLD2(x,mu_a_x,mu_a_y,d_r_a,sigma_d_r_a):    
    #sigma_d_r_a = .01
    # x[2] = 1
    d = 0
    for i in np.arange(start=0,stop=mu_a_x.__len__(),step=1):
        d = d+ (-2*d_r_a[i]*np.sqrt(x[2]**2*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2/(2*x[2]**2))+
        np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_a[i])
        # a = (2*d_r_a[i]*np.sqrt(x[2]*np.pi/2)*
        # hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2/(2*x[2])))
        # print(np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2)
    return d

def KLD3(x,mu_m_x,mu_m_y,sigma_m,d_r_m,sigma_d_r_m):    
    d = 0
    # x[2] = 1
    for i in np.arange(start=0,stop=mu_m_x.__len__(),step=1):
        d = d+(-2*d_r_m[i]*np.sqrt((x[2]**2+sigma_m[i])*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2/(2*(x[2]**2+sigma_m[i])))+
        np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_m[i])
    return d

def new_func2(func1, func2):
    return lambda x: func1(x) + func2(x)

def new_func3(func_List):
    return lambda x: sum(  f(x) for f in func_List  )

def sub_fuc_loc(balloon,i,dis,Sigma,iter_all):
            
    if balloon[i].A is False and balloon[i].convergen == False:
        KLD_local= lambda x: KLD1(x,balloon[i].mu,c)

        XX, YY, mu_m_x, mu_m_y, distance_a, distance_m, sigma_m, sigma_d_r_a, sigma_d_r_m = ([] for i in range(9))
                
        for j in np.arange(start=0,stop=balloon[i].neighbor.__len__(),step=1):
                    
            neighbor_label = balloon[i].neighbor[j]
            if neighbor_label in balloon[i].List:
                XX.append(balloon[neighbor_label].X)
                YY.append(balloon[neighbor_label].Y)
                distance_a.append(dis[i,neighbor_label])
                sigma_d_r_a.append(Sigma[i,j])
            else:         
                mu_m_x.append(balloon[neighbor_label].mu[0])
                mu_m_y.append(balloon[neighbor_label].mu[1])
                sigma_m.append(balloon[neighbor_label].Sigma)
                distance_m.append(dis[i,neighbor_label])
                sigma_d_r_m.append(Sigma[i,j])
                  
        KLD = lambda x: KLD3([x[0],x[1],x[2]],mu_m_x,mu_m_y,sigma_m,distance_m,sigma_d_r_m)+KLD2([x[0],x[1],x[2]],XX,YY,distance_a,sigma_d_r_a) +KLD_local([x[0],x[1],x[2]])
        a = balloon[i].mu[0]
        b = balloon[i].mu[1]
        c = balloon[i].Sigma

        # X = minimize(KLD,x0 = [a,b,c],method='nelder-mead'
        X = minimize(KLD,x0 = [a,b,c],method='TNC', bounds=((-np.inf, np.inf), (-np.inf, np.inf),(1, 100)))

        balloon[i].mu[0] = X.x[0]
        balloon[i].mu[1] = X.x[1]
        balloon[i].mu_x[iter_all] = X.x[0]
        balloon[i].mu_y[iter_all] = X.x[1]
        if iter_all>10:
            if np.abs(balloon[i].mu_x[iter_all]-balloon[i].mu_x[iter_all-1])<0.0001 and np.abs(balloon[i].mu_y[iter_all]-balloon[i].mu_y[iter_all-1])<0.0001:
                balloon[i].convergen = True
        balloon[i].Sigma = (X.x[2])
    return balloon

def localization(balloon,dis,Sigma,Leader):
    n = balloon.__len__() 
    for i in np.arange(start = 0, stop = n, step = 1):
        if i in GlobalVals.ANCHOR_LIST:
            balloon[i].mu = np.array([balloon[i].X , balloon[i].Y])
            balloon[i].Sigma = 10
        else:
            balloon[i].mu = np.array([balloon[i].X + np.random.randint(-100, 100), balloon[i].Y + np.random.randint(-100, 100)])
            # balloon[i].mu = np.array([balloon[i].X , balloon[i].Y ])
            balloon[i].Sigma = 500^2
        
        balloon[i].mu_x = np.zeros(GlobalVals.ITERATION)
        balloon[i].mu_y = np.zeros(GlobalVals.ITERATION)
        
        balloon[i].convergen = False
 
    #dis = dis1
    for iter_all in np.arange(start=0,stop=GlobalVals.ITERATION,step=1):

        def function(i): return sub_fuc_loc(balloon,i,dis,Sigma,iter_all)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=n)
        with executor:
            Q = {executor.submit(function, i) for i in np.arange(start=0, stop=n, step=1)}
        i=0
        Conve = 1
        for fut in concurrent.futures.as_completed(Q):
            balloon[i] = fut.result()[i]
            if balloon[i].convergen == True:
               Conve = Conve+1
            i = i+1
        if Conve == n:
           break

    for i in np.arange(start=0,stop=n,step=1):
        balloon[i].convergen = False
    
    p = np.zeros((n, 2))
    sigma = np.zeros((n, 1))        # covariance of the localisation algorithm
    for i in np.arange(start=0, stop=n, step=1):
        if i == Leader:
            p[i, :] = np.array([balloon[i].X, balloon[i].Y])
            sigma[i] = balloon[i].Sigma
        else:
            p[i, :] = np.array([balloon[i].mu[0], balloon[i].mu[1]])
            sigma[i] = balloon[i].Sigma
    return p,sigma,iter_all

def get_distance(n,balloon,offset,Leader,var_measurement,rssiAll,gps,gpsAll):
    distance_matrix = np.zeros((n,n))
    sigma = np.zeros((n,n))

    for i in np.arange(start=0,stop=n,step=1):
        if i == Leader:
            X = np.array([balloon[i].X,balloon[i].Y])+offset
        else:
            X = np.array([balloon[i].X,balloon[i].Y])

        for jj in np.arange(start=i,stop=n,step=1):
            
            Y = np.array([balloon[jj].X,balloon[jj].Y])
            if i in GlobalVals.REAL_BALLOON_LIST and jj in GlobalVals.REAL_BALLOON_LIST:
                test = distance2D([gps, gpsAll[jj],GlobalVals.GPS_REF,rssiAll[i][jj].distance])
                distance_matrix[i,jj]= distance2D([gps, gpsAll[jj],GlobalVals.GPS_REF,rssiAll[i][jj].distance])
                sigma[i,jj] = var_measurement
                sigma[jj,i] = var_measurement
            else:
                test = np.linalg.norm(X-Y) + random.uniform(-10,10)
                distance_matrix[i,jj] = np.linalg.norm(X-Y) + random.uniform(-10,10)  # CHECK THIS!! SHOULD WE ADD NOISE TO MEASUREMENT?
                sigma[i,jj] = 5
                sigma[jj,i] = 5
            
            distance_matrix[jj,i] = distance_matrix[i,jj]
            
    return distance_matrix,sigma

class balloon_class:
    pass

def balloon_main(Leader,anchor_list,positionXYZ,sigma_range_measurement_val,rssiAll,gps,gpsAll):
    n = GlobalVals.N_BALLOON
    balloon = [0] * n
    loc = np.zeros((n,2))

    for i in np.arange(start = 0, stop = n, step = 1):
        balloon[i]=balloon_class()
        balloon[i].X = positionXYZ[i].x
        balloon[i].Y = positionXYZ[i].y

        loc[i,:] = np.array([balloon[i].X,balloon[i].Y])
        
        balloon[i].mu = np.array([balloon[i].X, balloon[i].Y])
        balloon[i].Sigma = 1
        balloon[i].mu_x = np.zeros(GlobalVals.ITERATION)
        balloon[i].mu_y = np.zeros(GlobalVals.ITERATION)

        if i == Leader or i in anchor_list:
            balloon[i].A = True
        else:
            balloon[i].A = False
            balloon[i].neighbor = np.array(np.arange(start=0,stop=n,step=1))
            balloon[i].neighbor = np.delete(balloon[i].neighbor,i)
            balloon[i].List = anchor_list
            balloon[i].convergen = False

    offset = [0,0]
    dis,sigma_range_measurement = get_distance(n,balloon,offset,Leader,sigma_range_measurement_val,rssiAll,gps,gpsAll)    # noise sigma   
    p1,sigma1,iteration = localization(balloon,dis,sigma_range_measurement,Leader) # sigma of estimation



    return p1,sigma1,iteration,dis