#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    # # exp1
    # cfs = ["cf1", "cf5", "cf6", "cf7"]
    # waypoints = [
    #     [1.0, 0.0, 0.5, 3.0, 0.0], # x,y,z,time,startdelay
    #     [0.0, 1.0, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    #     [-1.0, -1.0, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    #     [-1.0, 1.0, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    # ]

    # # exp2
    # cfs = ["cf1", "cf5", "cf6", "cf7", "cf8"]
    # waypoints = [
    #     [-1.0, 0.0, 0.5, 3.0, 0.0], # x,y,z,time,startdelay
    #     [1.0, -1.0, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    #     [1.0, 1.0, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    #     [-1.0, -1.8, 0.5, 3.0, 0.5], # x,y,z,time,startdelay
    #     [-1.0, 1.5, 0.5, 3.0, 0.5], # x,y,z,time,startdelay

    # ]

    # exp3
    cfs = ["cf1", "cf5", "cf6", "cf7", "cf8"]
    waypoints = [
        [0.1, 0.0, 0.5, 2.0, 0.0], # x,y,z,time,startdelay
        [0.7, -1.0, 0.5, 2.0, 0.1], # x,y,z,time,startdelay
        [0.7, 1.0, 0.5, 2.0, 0.1], # x,y,z,time,startdelay
        [-0.7, -1.8, 0.5, 2.0, 0.1], # x,y,z,time,startdelay
        [-0.7, 1.5, 0.5, 2.0, 0.1], # x,y,z,time,startdelay

    ]

    allcfs = swarm.allcfs

    for cf in cfs:
        allcfs.crazyfliesByName[cf].takeoff(targetHeight=0.5, duration=2.0)
    timeHelper.sleep(2.5)
    
    # hover at initial position
    for cf in cfs:
        pos = np.array(allcfs.crazyfliesByName[cf].initialPosition) + np.array([0.0, 0.0, 0.5])
        allcfs.crazyfliesByName[cf].goTo(pos, 0, 1.0)
    timeHelper.sleep(1.5)

    # wait until main drone is at target height
    while True:
        if allcfs.crazyfliesByName["cf3"].position()[2] > 0.45:
            break
        timeHelper.sleep(0.1)
        
    for _ in range(2):
        # go to waypoint
        for cf, wp in zip(cfs, waypoints):
            timeHelper.sleep(wp[4])
            allcfs.crazyfliesByName[cf].goTo(wp[0:3], 0, wp[3])

        # timeHelper.sleep(max(wp[3] for wp in waypoints))
        timeHelper.sleep(1.0)

        # hover at initial position
        for cf, wp in zip(cfs, waypoints):
            timeHelper.sleep(wp[4])
            pos = np.array(allcfs.crazyfliesByName[cf].initialPosition) + np.array([0.0, 0.0, 0.5])
            allcfs.crazyfliesByName[cf].goTo(pos, 0, wp[3])
        # timeHelper.sleep(max(wp[3] for wp in waypoints))
        timeHelper.sleep(1.0)

    timeHelper.sleep(3.0)

    for cf in cfs:
        allcfs.crazyfliesByName[cf].land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)



if __name__ == '__main__':
    main()
