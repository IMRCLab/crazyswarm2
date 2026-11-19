#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    Z = 0.5

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Crazyflie 2.1 Brushless (and Bolt-based drones) must be armed before flight.
    # Either arm using the joystick or uncomment the following lines.
    # allcfs.arm(True)
    # timeHelper.sleep(1.0)

    for cf in allcfs.crazyflies:
        cf.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    relative_positions = [
        [0, 0, Z],
        [0.5, 0, Z],
        [0.5, 0.5, Z],
        [0.0, 0.5, Z],
        [0.0, 0.0, Z],
    ]

    for relative_position in relative_positions:
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array(relative_position)
            cf.goTo(pos, 0, 3.0)
        timeHelper.sleep(3.5)

    for cf in allcfs.crazyflies:
        cf.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

    # Disarm, if armed above (Crazyflie 2.1 Brushless and Bolt-based drones).
    # allcfs.arm(False)


if __name__ == '__main__':
    main()
