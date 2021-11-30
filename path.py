import numpy as np
from math import pi
import matplotlib.pyplot as plt

desc='''Waypoint navigation vs continuous curve tracking'''

def waypoints(N, A, B, a, b):
    ### Generates N points along the parametric lassajuous
    
    Period = 16 * pi                     ## This parameters control how far the points are sampled
    trajx = [A*np.sin(a*i)  for i in np.arange(0,Period, Period/(N+1))]
    trajy = [B*np.cos(b*i) for i in np.arange(0,Period, Period/(N+1))]
    return trajx, trajy

def main():
    print('Start')
    a = 1
    b = 1
    A = 2
    B = 2
    
    Xpoints, Ypoints  = [], []
    for i in [9]:    ## list of integers here
        b = i
        X,Y = waypoints(150, A,B,a,b)
        Xpoints += X
        Ypoints += Y
        
    stride = 1        
    plt.plot(Xpoints[::stride],Ypoints[::stride], 'r')
    #plt.savefig('./ABab_{}{}{}{}.png'.format(A,B,a,b))   ## Uncomment to save to file
    plt.show()
    plt.clf()
if __name__ == '__main__':
    main()
