import numpy as np
import matplotlib.pyplot as plt


data_vis=np.loadtxt('visionSyncedData50Hz.txt')
data_xs=np.loadtxt('xsensSyncedData.txt')

vis_x = data_vis[:,0]-data_vis[0,0]
vis_y = data_vis[:,1]-data_vis[0,1]
# xs_u = data_xs[::2,0]
# xs_v = data_xs[::2,1]
# xs_a = data_xs[::2,2]
xs_u = data_xs[::,0]
xs_v = data_xs[::,1]
xs_a = data_xs[::,2]


f = 100
delt = 0.01
F = np.array([[1,0,delt,0],[0,1,0,delt],[0,0,1,0],[0,0,0,1]])
H = np.array([[1,0,0,0],[0,1,0,0]])
B = np.array([[0,0,delt,0],[0,0,0,delt]])

x_pred = np.array([[0],[0],[0],[0]])
cx_pred = np.diag([1.0,1.0,0.1,0.1])
x_upd = np.array([[0],[0],[0],[0]])

var_z = 0.01**2 # camera noise
var_a = 0.01**2 # acceleration variance
var_vel = 0.2**2 # velocity noise
var_theta = 0.01**2 # angle noise
var_pos = 0.005**2 # position noise

cn = np.diag([var_z,var_z])#variance camera position
cw = np.diag([var_pos,var_pos,var_vel,var_vel])#covariance of prediction
cu = np.diag([var_a,var_a,var_theta])#covariance of acceleration

vis = (np.column_stack((vis_x,vis_y)))
xs = (np.column_stack((xs_u,xs_v,xs_a)))

data = np.zeros((2000,4))

for i in range(1000):
    S = np.dot(np.dot(H,cx_pred),H.transpose())+cn
    K = np.dot(np.dot(cx_pred,H.transpose()),np.linalg.inv(S))
    # zk = np.array([[0],[0]])
    zk=np.array([[vis[i,0]],[vis[i,1]]])
    x_upd = x_pred + np.dot(K,(zk-np.dot(H,x_pred)))
    cx_upd = cx_pred - np.dot(np.dot(K,S),K.transpose())

    uk=np.array([[xs[2*i,0]],[xs[2*i,1]],[xs[2*i,2]]])
    # uk=np.array([[xs[i,0]],[xs[i,1]],[xs[i,2]]])
    c,s=np.cos(uk[2]),np.sin(uk[2])
    thi,fou=delt*(uk[0]*c-uk[1]*s),delt*(uk[0]*s+uk[1]*c)
    gfun = np.array([[0],[0],[thi],[fou]])
    x_pred = np.dot(F,x_upd)+gfun
    for j in range(4):
        data[2*i,j]=x_pred[j]
        # data[i,j]=x_pred[j]
    G=np.array([[0,0,0],[0,0,0],[c,-s,-uk[0]*s-uk[1]*c],[s,c,uk[0]*c-uk[1]*s]])
    cx_pred=np.dot(np.dot(F,cx_upd),F.transpose())+np.dot(np.dot(G,cu),G.transpose())+cw

    S1 = np.dot(np.dot(H,cx_pred),H.transpose())+cn
    K1 = np.dot(np.dot(cx_pred,H.transpose()),np.linalg.inv(S))
    # zk = np.array([[0],[0]])
    # zk=np.array([[vis[i,0]],[vis[i,1]]])
    x_upd = x_pred + np.dot(K1,(zk-np.dot(H,x_pred)))
    cx_upd = cx_pred - np.dot(np.dot(K1,S1),K1.transpose())

    u=np.array([[xs[2*i+1,0]],[xs[2*i+1,1]],[xs[2*i+1,2]]])
    c1,s1=np.cos(u[2]),np.sin(u[2])
    t1,f1=delt*(u[0]*c1-u[1]*s1),delt*(u[0]*s1+u[1]*c1)
    gfun1 = np.array([[0],[0],[t1],[f1]])
    x_pred = np.dot(F,x_upd)+gfun1
    for j in range(4):
        data[2*i+1,j]=x_pred[j]
    G1=np.array([[0,0,0],[0,0,0],[c1,-s1,-u[0]*s1-u[1]*c1],[s1,c1,u[0]*c1-u[1]*s1]])
    cx_pred=np.dot(np.dot(F,cx_upd),F.transpose())+np.dot(np.dot(G1,cu),G1.transpose())+cw

#####################
#    figure plot    #
#####################
vx,vy=np.zeros([1001]),np.zeros([1001])
for i in range(1000):
    vx[i]=50*(vis_x[i+1]-vis_x[i])
    vy[i]=50*(vis_y[i+1]-vis_y[i])
t_1000=range(1001)
t_2000=range(2000)
fig = plt.figure()
plt.subplot(2,1,1)
plt.plot(t_2000,data[:,2],t_2000,data[:,3])
# plt.title('prediction position')
# plt.xlabel('time/us')
# plt.ylabel('position(m)')

plt.subplot(2,1,2)
plt.plot(t_1000,vx,t_1000,vy)
# plt.title('camera position')
# plt.xlabel('time/us')
# plt.ylabel('position(m)')
# plt.savefig('position.png')
plt.show()
