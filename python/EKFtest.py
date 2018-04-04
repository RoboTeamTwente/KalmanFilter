import numpy as np
import matplotlib.pyplot as plt

data_vis=np.loadtxt('visionRelevantData.txt')
data_xs=np.loadtxt('xsensRelevantData.txt')
# print(data_vis.shape,data_xs.shape)
f_xs = 100
f_vis = 50

v_xs = data_xs[:,0]
u_xs = data_xs[:,1]
a_xs = data_xs[:,2]/180*np.pi
t_xs = np.arange(len(data_xs))/f_xs
# print(v_xs.shape,u_xs.shape,a_xs.shape)
# print(t_xs.shape)
x_vis = data_vis[:,0]-data_vis[0,0]
y_vis = data_vis[:,1]-data_vis[0,1]
a_vis = data_vis[:,2]-data_vis[0,2]
t_vis = np.arange(len(data_vis))/f_vis

f_cam,f_xs=50,100
u=(np.vstack((u_xs,v_xs,a_xs)))
z=(np.vstack((x_vis,y_vis)))

# print(u.shape,u_xs.shape)
#####################
# define variance   #
#####################
var_a = 0.01**2 #acceleration
var_theta = 0.01**2#angle
var_xpos = 0.005**2#position
var_xvel = 0.005**2#velocity
var_z = 0.01**2#camera position

cu=np.diag([var_a,var_a,var_z])
cn=np.diag([var_z,var_z])
cw=np.diag([var_xpos,var_xpos,var_xvel,var_xvel])

H=np.array([[1,0,0,0],[0,1,0,0]])
x_pred = np.array([[0],[0],[0],[0]])
cx_pred = np.diag([1.0,1.0,0.1,0.1])

delay = 0.08
N_ahead=(int) (delay/(1/f_cam))

for k in range(1+N_ahead,len(x_vis)):
    zk=z[:,k]
    if(np.isnan(zk[1])):
        x_upd=x_pred
        cx_upd=cx_pred
    else:
        S=np.dot(np.dot(H,cx_pred),H.transpose())+cn
        K=np.dot(np.dot(cx_pred,H.transpose()),np.linalg.inv(S))
        x_upd=x_pred+np.dot(K,(zk-np.dot(H,x_pred)))
        cx_upd=cx_pred-np.dot(np.dot(K,S),K.transpose())

        uk=u[:,k-N_ahead]
        
#####################
#    figure plot    #
#####################

# fig = plt.figure()
# plt.subplot(1,2,1)
# plt.plot(t_vis,x_vis,t_vis,y_vis)
# plt.title('vision')
# plt.xlabel('time/us')
# plt.ylabel('position/m')
# # plt.show()
#
# plt.subplot(1,2,2)
# plt.plot(t_xs,v_xs,t_xs,u_xs)
# plt.title('xsens')
# plt.xlabel('time/us')
# plt.ylabel('acceleration(m^2/s)')
# plt.show()
