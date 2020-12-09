import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
import copy

def cluster():
    g0=np.random.multivariate_normal([37.568817, 126.978966], [[0.000005,0],[0,0.000005]], 100)
    g1=np.random.multivariate_normal([37.568817, 126.978966], [[0.000005,0],[0,0.000005]], 100)
    g2=np.random.multivariate_normal([37.568628, 126.978801], [[0.000005,0],[0,0.000005]], 100)
    x=np.vstack([g0, g1, g2])
    x=np.asmatrix(x)
    print(x)
  
    k=2
    m=x.shape[0]

    mu = x[np.random.randint(0, m, k), :]
    print(mu)
  
    samecount = 0
    y = np.empty([m,1])
    for j in range(100):    
        for i in range(m):
            d0=np.linalg.norm(x[i,:]-mu[0,:],2)
            d1=np.linalg.norm(x[i,:]-mu[1,:],2)
            # d2=np.linalg.norm(x[i,:]-mu[2,:],2)
            y[i]=np.argmin([d0, d1])
        for i in range(k):
            mu[i, :] = np.mean(x[np.where(y==i)[0]], axis = 0)
        print("current step is... ", j)
        print(mu)


    x0 = x[np.where(y==0)[0]]
    x1 = x[np.where(y==1)[0]]
  
    print("result \n\n")
    output1=open('./drone1.txt','w')
    output1.write(str(float(mu[0][:,0]))+'\n')
    output1.write(str(float(mu[0][:,1])))
    output1.close()
    output2=open('./drone2.txt','w')
    output2.write(str(float(mu[1][:,0]))+'\n')
    output2.write(str(float(mu[1][:,1])))
    output2.close()
    print(mu)
    return 0

 

cluster()
