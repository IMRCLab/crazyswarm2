#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    name1 = "bb01"
    name2 = "aa01"
    Pos1 = np.array([0.0, -0.2, 0.0])
    Pos2 = np.array([0.0, 0.2, 0.0])
    Height1 = 0.4
    Height2 = 0.5
    swapTime = 3

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.arm(True)
    timeHelper.sleep(1.0)

    allcfs.takeoff(targetHeight=Height1, duration=3.0)
    timeHelper.sleep(3.5)

    # go to initial positions
    allcfs.crazyfliesByName[name1].goTo(Pos1 + np.array([0, 0, Height1]), 0, 3.0)
    allcfs.crazyfliesByName[name2].goTo(Pos2 + np.array([0, 0, Height2]), 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.setParam('ctrlLee.indi', 1)

    # swap 1
    allcfs.crazyfliesByName[name1].goTo(Pos2 + np.array([0, 0, Height1]), 0, swapTime)
    allcfs.crazyfliesByName[name2].goTo(Pos1 + np.array([0, 0, Height2]), 0, swapTime)
    timeHelper.sleep(swapTime + 1.5)

    # swap 2
    allcfs.crazyfliesByName[name1].goTo(Pos1 + np.array([0, 0, Height1]), 0, swapTime)
    allcfs.crazyfliesByName[name2].goTo(Pos2 + np.array([0, 0, Height2]), 0, swapTime)
    timeHelper.sleep(swapTime + 1.5)

    allcfs.setParam('ctrlLee.indi', 0)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.5)
    allcfs.arm(False)
    timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
