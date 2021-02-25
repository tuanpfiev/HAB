import numpy as np
import concurrent.futures
from scipy.stats import hypergeom
from scipy.special import hyp1f1
from scipy.io import loadmat
from scipy.optimize import fmin,fmin_powell,fmin_cg,minimize
import inspect
from scipy import random
from  Ambi_Resolve_A import Ambi_resolve


def KLD1(x,mu_r,sigma_r):
    if abs(sigma_r)<1e-6:
        sigma_r = 1e-6
    return 1/2*(np.linalg.norm(np.array([x[0]-mu_r[0],x[1]-mu_r[1]]))**2+2*x[2]**2)/sigma_r-np.log(x[2]**2/sigma_r)-1

def KLD2(x,mu_a_x,mu_a_y,d_r_a,sigma_d_r_a):    
    #sigma_d_r_a = .01
    d = 0
    for i in np.arange(start=0,stop=mu_a_x.__len__(),step=1):
        d = d+ (-2*d_r_a[i]*np.sqrt(x[2]**2*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2/(2*x[2]**2))+
        np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_a[i])
    return d

def KLD3(x,mu_m_x,mu_m_y,sigma_m,d_r_m,sigma_d_r_m):    
    
    d = 0
    for i in np.arange(start=0,stop=mu_m_x.__len__(),step=1):
        d = d+(-2*d_r_m[i]*np.sqrt((x[2]**2+sigma_m[i])*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2/(2*(x[2]**2+sigma_m[i])))+
        np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_m[i])
    return d

def KLD3_1(x,mu_m_x,mu_m_y,sigma_m,d_r_m):    
    sigma_d_r_m = .01
    return (-2*d_r_m*np.sqrt((x[2]**2+sigma_m)*np.pi/2)*
    hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_m_x,x[1]-mu_m_y]))**2/(2*(x[2]**2+sigma_m)))+
    np.linalg.norm(np.array([x[0]-mu_m_x,x[1]-mu_m_y]))**2+2*x[2]**2)/(2*sigma_d_r_m)


def new_func2(func1, func2):
    return lambda x: func1(x) + func2(x)

def new_func3(func_List):
    return lambda x: sum(  f(x) for f in func_List  )

def sub_fuc_loc(balloon,i,dis,Sigma,iter_all):
            
    if balloon[i].A is False and balloon[i].convergen == False:
        KLD_local= lambda x: KLD1(x,balloon[i].mu,b)

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
                  
        KLD = lambda x: KLD3(x,mu_m_x,mu_m_y,sigma_m,distance_m,sigma_d_r_m)+KLD2(x,XX,YY,distance_a,sigma_d_r_a) +KLD_local(x)
        
        a = balloon[i].mu[0]
        b = balloon[i].mu[1]
        c = balloon[i].Sigma**(1/2)
        
        X = minimize(KLD,x0 = [a,b,c],method='nelder-mead', options={'xatol': 0.001, 'fatol': 0.001})
        
        balloon[i].mu[0] = X.x[0]
        balloon[i].mu[1] = X.x[1]
        balloon[i].mu_x[iter_all] = X.x[0]
        balloon[i].mu_y[iter_all] = X.x[1]
        if iter_all>10:
            if np.abs(balloon[i].mu_x[iter_all]-balloon[i].mu_x[iter_all-1])<0.0001 and np.abs(balloon[i].mu_y[iter_all]-balloon[i].mu_y[iter_all-1])<0.0001:
                balloon[i].convergen = True
        balloon[i].Sigma = np.abs(X.x[2])
    return balloon

def localization(balloon,dis,Sigma,Leader):
    n = balloon.__len__() 
    for i in np.arange(start = 0, stop = n, step = 1):
        balloon[i].mu = np.array([balloon[i].X + np.random.randint(-10, 10), balloon[i].Y + np.random.randint(-10, 10)])
        balloon[i].mu_x = np.zeros(250)
        balloon[i].mu_y = np.zeros(250)
        balloon[i].Sigma = 1
        balloon[i].convergen = False
 
    #dis = dis1
    for iter_all in np.arange(start=0,stop=50,step=1):

        def function(i): return sub_fuc_loc(balloon,i,dis,Sigma,iter_all)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=n)
        with executor:
            Q = {executor.submit(function, i) for i in np.arange(start=1, stop=n, step=1)}
        i=1
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

def get_distance(n,balloon,offset,Leader,var_measurement):
    distance_matrix = np.zeros((n,n))
    sigma = np.zeros((n,n))

    for i in np.arange(start=0,stop=n,step=1):
        if i == Leader:
            X = np.array([balloon[i].X,balloon[i].Y])+offset
        else:
            X = np.array([balloon[i].X,balloon[i].Y])

        for jj in np.arange(start=i,stop=n,step=1):
            
            Y = np.array([balloon[jj].X,balloon[jj].Y])
            distance_matrix[i,jj] = np.linalg.norm(X-Y) 
            sigma[i,jj] = var_measurement
            sigma[jj,i] = var_measurement
            distance_matrix[i,jj] =distance_matrix[i,jj]+ random.uniform(0,10)
            distance_matrix[jj,i] = distance_matrix[i,jj]
            
    return distance_matrix,sigma

class balloon_class:
    pass

def balloon_main(n,Leader,anchor_list,positionXY,sigma_range_measurement_val):
    balloon = [0] * n
    loc = np.zeros((n,2))

    for i in np.arange(start = 0, stop = n, step = 1):
        balloon[i]=balloon_class()
        balloon[i].X = positionXY[i,0]
        balloon[i].Y = positionXY[i,1]

        loc[i,:] = np.array([balloon[i].X,balloon[i].Y])
        
        balloon[i].mu = np.array([balloon[i].X, balloon[i].Y])
        balloon[i].Sigma = 1
        balloon[i].mu_x = np.zeros(250)
        balloon[i].mu_y = np.zeros(250)

        if i == Leader:
            balloon[i].A = True
        else:
            balloon[i].A = False
            balloon[i].neighbor = np.array(np.arange(start=0,stop=n,step=1))
            balloon[i].neighbor = np.delete(balloon[i].neighbor,i)
            balloon[i].List = anchor_list
            balloon[i].convergen = False

    offset = [0,0]
    dis,sigma_range_measurement = get_distance(n,balloon,offset,Leader,sigma_range_measurement_val)    # noise sigma   
    p1,sigma1,iteration = localization(balloon,dis,sigma_range_measurement,Leader) # sigma of estimation


    # offset = [0,1]
    # dis,Sigma = get_distance(balloon,offset,Leader) 
    # p2,sigma2 = localization(balloon,dis,Sigma,Leader)

    # offset = [1,0]
    # dis,Sigma = get_distance(balloon,offset,Leader) 
    # p3,sigma3 = localization(balloon,dis,Sigma,Leader)
    # offset = [0,0]
    # p1,sigma1 = localization.localization(balloon,0,offset)
    # offset = [0,1]
    # p2,sigma2 = localization.localization(balloon,0,offset)
    # offset = [1,0]
    # p3,sigma3 = localization.localization(balloon,0,offset)
    # offset = np.array([[ 0, 1],[ 1, 0]])
    # temp = Ambi_resolve(p1,p2,p3,offset)+np.array([balloon[0].X,balloon[0].Y])
    # offset = np.array([[ 0, 1],[ 1, 0]])
    # temp = Ambi_resolve.Ambi_resolve(p1,p2,p3,offset)+np.array([balloon[0].X,balloon[0].Y])

    # print(loc)
    # print(p1)
    # error1 = np.array(([p1[0,0]-loc[0,0],p1[0,1]-loc[0,1]],[p1[1,0]-loc[1,0],p1[1,1]-loc[1,1]],[p1[2,0]-loc[2,0],p1[2,1]-loc[2,1]],[p1[3,0]-loc[3,0],p1[3,1]-loc[3,1]],[p1[4,0]-loc[4,0],p1[4,1]-loc[4,1]]))
    # error2 = np.array([[p2[0,0]-loc[0,0],p2[0,1]-loc[0,1]],[p2[1,0]-loc[1,0],p2[1,1]-loc[1,1]],[p2[2,0]-loc[2,0],p2[2,1]-loc[2,1]],[p2[3,0]-loc[3,0],p2[3,1]-loc[3,1]],[p2[4,0]-loc[4,0],p1[4,1]-loc[4,1]]])
    # error3 = np.array([[p3[0,0]-loc[0,0],p3[0,1]-loc[0,1]],[p3[1,0]-loc[1,0],p3[1,1]-loc[1,1]],[p3[2,0]-loc[2,0],p3[2,1]-loc[2,1]],[p3[3,0]-loc[3,0],p3[3,1]-loc[3,1]],[p3[4,0]-loc[4,0],p1[4,1]-loc[4,1]]])
    # print('error 1 .....')
    # print(loc-p1)
    # print('error 2 .....')
    # print(error2)
    # print('error 3 .....')
    # print(error3)        
    # print('3 anchors.....')
    # print(temp-loc)


    return p1,sigma1,iteration