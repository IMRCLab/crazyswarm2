#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

from crazyflie_interfaces.msg import LogDataGeneric

rpms1 = []
rpms2 = []

def rpm_callback1(msg):
    rpms1.append(msg.values)

def rpm_callback2(msg):
    rpms2.append(msg.values)

def main():
    name1 = "bb01"
    name2 = "aa01"
    weight1 = 0.0427 #kg
    weight2 = 0.0364 #kg
    Pos1 = np.array([0.0, -0.2, 0.0])
    Pos2 = np.array([0.0, 0.2, 0.0])
    Height1 = 0.4
    Height2 = 0.5
    swapTime = 3

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    rpmSubscriber1 = allcfs.create_subscription(
        LogDataGeneric, f'{name1}/rpm', rpm_callback1, 10)
    rpmSubscriber2 = allcfs.create_subscription(
        LogDataGeneric, f'{name2}/rpm', rpm_callback1, 10)

    allcfs.arm(True)
    timeHelper.sleep(1.0)

    allcfs.takeoff(targetHeight=Height1, duration=3.0)
    timeHelper.sleep(3.5)

    # go to initial positions
    allcfs.crazyfliesByName[name1].goTo(Pos1 + np.array([0, 0, Height1]), 0, 3.0)
    allcfs.crazyfliesByName[name2].goTo(Pos2 + np.array([0, 0, Height2]), 0, 3.0)
    timeHelper.sleep(3.5)

    # calibrate
    rpms1.clear()
    rpms2.clear()
    timeHelper.sleep(2.0)
    rpms1_np = np.array(rpms1)
    rpms2_np = np.array(rpms2)
    # sanity check that all sensors work as expected
    print(rpms1_np.mean(0))
    print(rpms1_np.mean(1))
    print(rpms1_np.std(0))
    print(rpms1_np.std(1))

    omega1 = rpms1_np.mean() / 60 * 2 * np.pi
    omega2 = rpms2_np.mean() / 60 * 2 * np.pi

    # f = kappa_f * omega1^2
    kappa_f1 = weight1 * 9.81 / 4 / omega1**2
    print(kappa_f1)
    kappa_f2 = weight1 * 9.81 / 4 / omega2**2

    allcfs.crazyfliesByName[name1].setParam('ctrlLee.kappa_f', kappa_f1)
    allcfs.crazyfliesByName[name2].setParam('ctrlLee.kappa_f', kappa_f2)

    timeHelper.sleep(2.0)

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
