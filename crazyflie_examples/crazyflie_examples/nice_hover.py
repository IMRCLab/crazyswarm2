#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    Z = 0.5
    print("here 0")
    swarm = Crazyswarm()
    print("here 1")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    print("here 2")

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    print("here 3")
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)

    print('press button to start turn ...')
    swarm.input.waitUntilButtonPressed()
        
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, np.pi/3, 1.0)
        
    
    print('press button to land ...')
    swarm.input.waitUntilButtonPressed()
    

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)


if __name__ == '__main__':
    main()
