from crazyflie_py import Crazyswarm
import numpy as np
import time


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf1 = swarm.allcfs.crazyflies[0]
    cf2 = swarm.allcfs.crazyflies[1]
    
    Z = 1.0
    
    print(f"crazyflie 1 took off from {cf1.initialPosition}")
    cf1.takeoff(targetHeight=Z, duration=2.5)
    timeHelper.sleep(2.5)
    
    print(f"crazyflie 2 took off from {cf2.initialPosition}")
    cf2.takeoff(targetHeight=Z, duration=2.5)
    timeHelper.sleep(2.5)
    
    '''
    pos = np.array([1.0,1.0,Z])
    print(f"going to {pos} ... {time.time()}")
    cf.goTo(pos, np.pi/2, 2.0)
    timeHelper.sleep(2.5)
    
    pos = np.array([1.0,-1.0,Z])
    print(f"going to {pos} ... {time.time()}")
    cf.goTo(pos, np.pi/2, 2.0)
    timeHelper.sleep(2.5)
    
    pos = np.array([-1.0,-1.0,Z])
    print(f"going to {pos} ... {time.time()}")
    cf.goTo(pos, np.pi/2, 2.0)
    timeHelper.sleep(2.5)
    
    pos = np.array([-1.0,1.0,Z])
    print(f"going to {pos} ... {time.time()}")
    cf.goTo(pos, np.pi/2, 2.0)
    timeHelper.sleep(2.5)
    
    pos = np.array([0,0,Z])
    print(f"going to {pos} ... {time.time()}")
    cf.goTo(pos, np.pi/2, 2.0)
    timeHelper.sleep(2.5)
    '''
    
    print(f"landing now ... {time.time()}")

    cf1.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(2.5)
    
    cf2.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(2.5)

if __name__ == '__main__':
    main()
