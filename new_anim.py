import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import math
from scipy import integrate



###################################
## Plotting related initializations
###################################
#fig = plt.figure()
#fig.set_dpi(100)
#fig.set_size_inches(11, 9)
#
#ax = plt.axes(xlim=(-9, 9), ylim=(-9, 9))
fig_size=[0, 0]
fig_size[0] = 20
fig_size[1] = 8
plt.rcParams["figure.figsize"] = fig_size

fig, (ax, ax2) = plt.subplots(1,2)

patch1 = plt.Circle((10, -10), radius=0.3, fc='b', ec='r', alpha=0.4)
patch2 = plt.Circle((10, -10), radius=0.1, fc='r', ec='k', alpha=0.3)
patch3 = plt.Circle((10, -10), radius=0.3, fc='g', ec='k', alpha=0.5)

###################################
## Defining the unicycle 
###################################

class bot_unicycle(object):
    '''Bot uniclcle class represents a unicylcle model (Diffrential Equations and corresponding solver)'''
    def __init__(self, d, l, ic):
        self.d = d
        self.l = l
        self.X = ic
        self.t1 = 0.0
        print(self.X)

    def model(self, Y, t, v):
        ''' This is the differential equation function that will be 
        solved for the unicycle '''
        theta_e = Y[-1] 
        x_dot  = v[0]*np.cos(theta_e)
        y_dot  = v[0]*np.sin(theta_e)
        theta_dot = v[1]
        
        return [x_dot, y_dot, theta_dot]
        
    def step(self, dt, v):
        response = integrate.odeint(self.model, self.X, [self.t1, self.t1+dt], args=(v,))
        #print np.shape(response)

        #print response,'"'
        self.X = response[-1,:]
        #theta is not bounded in the above expression

        self.t1 = self.t1 + dt
        return self.X


###################################
##  Create a Unicycle instance
###################################
x_initial = 2
y_initial = -1
theta_initial = np.pi/3

agent = bot_unicycle(1,1,[x_initial, y_initial, theta_initial])

## Define input for the unicycle
v_input = [0.2, 0.1]
time_delta = 0.15

## Run 1 step 
cx, cy, t = agent.step(time_delta, v_input)


###################################
## Write the control law here
###################################

def mapTheta(x):
    ''' map the angle of the agent to [0, 2*pi]'''
    pi = np.pi
    if (x>=0 and x<=2*pi):
        return x
    if (x<0):
        y = -x
        x_mod = y%(2*pi)
        return ((2*pi)-x_mod)
    if (x>2*pi):
        return x%(2*pi)
    return 0


def sat(x,ma):
    if x>ma:
        x = ma
    if x<-ma:
        x = -ma
    return x

def controller(pose_ref, pose_cur, law=0):
    v = 0
    w = 0
    if law==0:
        print("proportional law")
    if law==1:
        ex = pose_cur[0] - pose_ref[0]
        ey = pose_cur[1] - pose_ref[1]
        
        #theta_ref = mapTheta(pose_ref[2])
        #theta_cur = mapTheta(pose_cur[2])
        theta_ref = mapTheta(pose_ref[2])
        theta_cur = mapTheta(pose_cur[2])
        print("Theta_ref :{:.4f}, Theta_cur {:.4f}".format(pose_ref[2], pose_cur[2]))
        et = (theta_cur - theta_ref)
        
        if(et>np.pi):
            et = -((2*np.pi)-et)
        if(et<-np.pi):
            et = et + (2*np.pi)

        D = (ex**2 + ey**2)**(0.5)
        K1 = 5
        K2 = 3
        v = K1*math.cos(et)*D
        w = -K2*et
        v = sat(v, 1)
        w = sat(w, 1)
        print("Theta_ref {:.4f}, Theta_cur {:.4f}, e {:.4f}, v {}, w {}".format(theta_ref, theta_cur, et, v, w)) 
    return [v,w], et

###################################


pi = np.pi
A = 7
B = 8
a = 1
b = 2

cx = 0
cy = 0
theta = 0

V =[0]
W =[0]
E =[0]
#patch = plt.Circle((5, -5), 0.75, fc='y')
line = []

def init():
    cx = 1
    cy = -10
    theta = 1.2   # in radians
    patch1.center = (cx, cy)
    ax.add_patch(patch1)
    ax.add_patch(patch2)
    ax.add_patch(patch3)
    for i in np.arange(0,2*np.pi, 2*np.pi/100):
        ax.plot(A*np.cos(a*i), B*np.sin(b*i), 'g.')

    ax.plot(A*np.cos(a*i), B*np.sin(b*i), 'g.', label="reference traj")
    line1, = ax2.plot(V, 'r', label='control 1')
    line2, = ax2.plot(W, 'g', label='control 2')
    line3, = ax2.plot(E, 'b', label='theta error')

    line.append(line1)
    line.append(line2)
    line.append(line3)
    
    plt.legend(loc='best')
    #patch2 = plt.Arrow(cx, cy, 0.4*math.cos(theta), 0.4*math.sin(theta), width=0.1)
    return patch1, patch2, patch3, 

x_ref = A
y_ref = 0
def animate(i):
    global v_input
    global cx, cy, theta
    global V,W,E
    
    j = i*2*np.pi/720
    
    x_ref = A*np.cos(a*j)
    y_ref = B*np.sin(b*j)
    #x_dot = -A*a*np.sin(a*j)
    #y_dot = B*b*np.cos(b*j)
    ey = cy - y_ref
    ex = cx - x_ref
    theta_ref = np.arctan2(-(ey), -(ex))
     
    
    ############## Calculate controller
    v_input, e = controller([x_ref, y_ref, theta_ref], [cx, cy, theta], 1)
    V.append(v_input[0])
    W.append(v_input[1])
    E.append(e)
    
    ############## ODE stepping
    cx, cy, theta = agent.step(time_delta, v_input)

    ############## Update the animation 
    patch1.center = (cx, cy)
    patch2.center = (cx+0.35*math.cos(theta), cy+0.35*math.sin(theta))
    patch3.center = (x_ref, y_ref)
    line[0].set_ydata(V)
    line[1].set_ydata(W)
    line[2].set_ydata(E)
    
   

    return patch1, patch2, patch3, 

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=720, 
                               interval=35,
                               blit=True)

plt.show()

