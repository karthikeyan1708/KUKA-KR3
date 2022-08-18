import numpy as np

#The DH parameters value 
a = np.array([-20,260,20,0,0,0])
d = np.array([-345,0,0,-260,0,-75])
alpha = np.array([-1.57,0,1.57,-1.57,1.57,3.14])
offsetvalue = np.array([1.57,3.14,-1.57,1.396,0,3.14])
dh_parameters = np.array([a,d,alpha,offsetvalue]).T
print (dh_parameters)

def Jacobian(dhparameters):
    Forward_Kinematics = ForwardKinematics(dhparameters)
    Z = Forward_Kinematics[1]
    t = Forward_Kinematics[2]
    J_v = []
    J_w = []
    
    for i in range(dhparameters.shape[0]):
        temp = Z[i]*(t[6]-t[i])
        J_v.append(temp)
        
    for j in range(dhparameters.shape[0]):
        J_w.append(Z[j])
    a_temp = np.zeros((6,1))
    
    for u in range(6):
        a = np.concatenate((J_v[u],J_w[u])).reshape(6,1)
        a_temp = np.concatenate((a_temp,a),axis = 1)
    Jacobian_matrix = a_temp[:,1::]
    
    return Jacobian_matrix

def ForwardKinematics(dhvalues):  
    
    DH = dhvalues
    HTM = np.zeros((4,4),dtype='i')
    HTM_List = []
    W1 = np.eye(4, dtype='i')
    P = []
    P.append(np.array([0,0,1]))
    t = []
    t.append(np.array([0,0,0]))
    
    for i in range(dhvalues.shape[0]):              
        HTM = np.array([[np.cos(DH[i,3]),
        -np.sin(DH[i,3])*np.cos(DH[i,1]), np.sin(DH[i,3])*np.sin(DH[i,1]), 
        DH[i,0]*np.cos(DH[i,3])], [np.sin(DH[i,3]), np.cos(DH[i,3])*np.cos(DH[i,1]), 
        -np.cos(DH[i,3])*np.sin(DH[i,1]), DH[i,0]*np.sin(DH[i,3])],
        [0, np.sin(DH[i,1]), np.cos(DH[i,1]), DH[i,2]],
        [0, 0, 0, 1]])
        HTM_List.append(HTM)
        
        W1 = np.matmul(W1,HTM)
        print(W1)
        P.append(W1[0:3,-2])
        print(len(P))
        t.append(W1[0:3,-1])
    return W1, P, t

J = Jacobian(dh_parameters)
print('Jacobian Matrix: \n',J)