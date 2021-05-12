import numpy as np

def g(x):
    return x - 2*np.pi*np.floor(x/(2*np.pi)+1/2)

def M(t): return np.array([[np.cos(t), -np.sin(t)], [np.sin(t),  np.cos(t)]])

def f(a,b,c,t): return np.sum((a+b*np.cos(t)+c*np.sin(t))**2)

def Ambi_resolve(p1,p2,p3,offset):
    dis = np.zeros((8, 8,3))
    n=5
    for i in np.arange(start=0, stop=n, step=1):
        p1 = p1 - p1[0,:]

    for i in np.arange(start=0, stop=n, step=1):
        for jj in np.arange(start=i, stop=n, step=1):
            dis[i, jj,0] = np.linalg.norm(p1[i, :]-p1[jj, :])**2
            dis[jj, i,0] = dis[i, jj,0]
    for i in np.arange(start=0, stop=n, step=1):
        for jj in np.arange(start=i, stop=n, step=1):
            dis[i, jj,1] = np.linalg.norm(p2[i, :]-p2[jj, :])**2
            dis[jj, i,1] = dis[i, jj,1]
    for i in np.arange(start=0, stop=n, step=1):
        for jj in np.arange(start=i, stop=n, step=1):
            dis[i, jj,2] = np.linalg.norm(p3[i, :]-p3[jj, :])**2
            dis[jj, i,2] = dis[i, jj,2]
    np.set_printoptions(precision=3,suppress=True)
    #print(dis[:, :,0])
    p1_temp = p1[:]
    for i in np.arange(start=0, stop=2, step=1):
        d_temp = dis[:,:,0]-dis[:,:,i+1]
        d = d_temp[0,1:]
        a = np.zeros(8)
        b = np.zeros(8)
        c = np.zeros(8)
        offset_temp = offset[i,:]
        for j in np.arange(start=1, stop=n, step=1):
            a[j] = offset_temp[0]**2+offset_temp[1]**2+d[j-1]
            b[j] = 2*(offset_temp[0]*(p1_temp[0,0]-p1_temp[j,0])+offset_temp[1]*(p1_temp[0,1]-p1_temp[j,1]))
            c[j] = 2*(offset_temp[1]*(-p1_temp[0,0]+p1_temp[j,0])+offset_temp[0]*(p1_temp[0,1]-p1_temp[j,1]))
        alpha = np.sum(a*b);
        beta = np.sum(a*c);
        gamma = np.sum(b*c);
        delta  = np.sum(c**2-b**2);
        A = 4*gamma**2+delta**2;
        B = 2*(2*alpha*gamma+beta*delta);
        C = alpha**2+beta**2-4*gamma**2-delta**2;
        D = 2*(-alpha*gamma-beta*delta);
        E = -beta**2+gamma**2;
        root_temp = np.roots([A,B,C,D,E])
        root = np.real(root_temp[np.imag(root_temp)==0])
        angle1 = np.arcsin(root);
        angle2 = np.pi-angle1
        angle = np.ndarray.flatten(np.array([angle1,angle2]))        
        L =[f(a,b,c,t) for t in angle]
        v = np.argmin(L)
        v = g(angle[v])       
        p1_temp = p1_temp.dot(M(v))
    tt = -2*np.arctan2(offset[0,0],offset[0,1])+2*np.arctan2(offset[1,0],offset[1,1])
    threshold = 1/2*(g(tt))
    flag = 0
    if np.abs(v)>abs(threshold):
        flag = 1;

    if flag:
        p1_temp = p1*[-1,1];
        for i in np.arange(start=0, stop=1, step=1):
            
            d_temp = dis[:,:,0]-dis[:,:,i+1]
            d = d_temp[0,1:]
            a = np.zeros(8)
            b = np.zeros(8)
            c = np.zeros(8)
            offset_temp = offset[i,:]
            for j in np.arange(start=1, stop=n, step=1):
                a[j] = offset_temp[0]**2+offset_temp[1]**2+d[j-1]
                b[j] = 2*(offset_temp[0]*(p1_temp[0,0]-p1_temp[j,0])+offset_temp[1]*(p1_temp[0,1]-p1_temp[j,1]))
                c[j] = 2*(offset_temp[1]*(-p1_temp[0,0]+p1_temp[j,0])+offset_temp[0]*(p1_temp[0,1]-p1_temp[j,1]))
            alpha = np.sum(a*b);
            beta = np.sum(a*c);
            gamma = np.sum(b*c);
            delta  = np.sum(c**2-b**2);
            A = 4*gamma**2+delta**2;
            B = 2*(2*alpha*gamma+beta*delta);
            C = alpha**2+beta**2-4*gamma**2-delta**2;
            D = 2*(-alpha*gamma-beta*delta);
            E = -beta**2+gamma**2;
            root_temp = np.roots([A,B,C,D,E])
            root = np.real(root_temp[np.imag(root_temp)==0])
            angle1 = np.arcsin(root);
            angle2 = np.pi-angle1
            angle = np.ndarray.flatten(np.array([angle1,angle2]))
            L =[f(a,b,c,t) for t in angle]
            v = np.argmin(L)
            v = angle[v]  
            #print(M(v))    
            p1_temp = p1_temp.dot(M(v))
    return p1_temp
