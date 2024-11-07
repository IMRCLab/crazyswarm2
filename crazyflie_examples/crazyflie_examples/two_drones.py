from crazyflie_py import Crazyswarm
import numpy as np
import time


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf1 = swarm.allcfs.crazyflies[1]
    cf2 = swarm.allcfs.crazyflies[0]
    
    Z = 1.0
    
    timeHelper.sleep(5)
    
    print(f"cfs taking off...")
    cf1.takeoff(targetHeight=Z, duration=2.5)
    cf2.takeoff(targetHeight=Z, duration=2.5)
    timeHelper.sleep(3)
    
    
    pos = np.array([1.0,1.0,Z])
    print(f"cf1 going to {pos} ... {time.time()}")
    cf1.goTo(pos, 0, 5.0)
    pos = np.array([1.0,-1.0,Z-0.5])
    print(f"cf2 going to {pos} ... {time.time()}")
    cf2.goTo(pos, 0, 5.0)
    timeHelper.sleep(5)
    
    print(f"cf1 going to {pos} ... {time.time()}")
    cf1.goTo(pos, 0, 5.0)
    pos = np.array([-1.0,-1.0,Z])
    print(f"cf2 going to {pos} ... {time.time()}")
    cf2.goTo(pos, 0, 5.0)
    timeHelper.sleep(5)
    
    print(f"cf1 going to {pos} ... {time.time()}")
    cf1.goTo(pos, 0, 5.0)
    pos = np.array([-1.0,1.0,Z-0.5])
    print(f"cf2 going to {pos} ... {time.time()}")
    cf2.goTo(pos, 0, 5.0)
    timeHelper.sleep(5)
    
    print(f"cf1 going to {pos} ... {time.time()}")
    cf1.goTo(pos, 0, 5.0)
    pos = np.array([1.0,1.0,Z])
    print(f"cf2 going to {pos} ... {time.time()}")
    cf2.goTo(pos, 0, 5.0)
    timeHelper.sleep(5)
    
    
    print(f"cfs landing now ... {time.time()}")
    cf1.land(targetHeight=0.04, duration=2.5)
    cf2.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(2.5)



if __name__ == '__main__':
    main()
  
