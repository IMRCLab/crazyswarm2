#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    TIMESCALE = 1.0
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)
    timeHelper.sleep(1)

    # pm_state : 0 = on battery  1 = charging  2 = charged  3 = low power  4 = shutdown
    flight_counter = 1

    while True:
        # Crazyflie 2.1 Brushless (and Bolt-based drones) must be armed before flight.
        # Either arm using the joystick or uncomment the following lines.
        # allcfs.arm(True)
        # timeHelper.sleep(1.0)

        print('takeoff')
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        # fly figure8 until battery is low
        fig8_counter = 0
        status = allcfs.crazyflies[0].get_status()
        # while status['battery'] > 3.8:
        while status['pm_state'] == 0:
            fig8_counter += 1
            print(f'starting figure8 number {fig8_counter} of flight number {flight_counter}')
            allcfs.startTrajectory(0, timescale=TIMESCALE)
            timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
            status = allcfs.crazyflies[0].get_status()
            print(f'pm state : {status["pm_state"]}, battery left : {status["battery"]}')
            timeHelper.sleep(1)

        # not sure if useful
        # check if pm = 3 just to be sure, if not abort test
        if status['pm_state'] != 3:
            print(f'power state is not 3 (low) but {status["pm_state"]}. Landing and aborting')
            allcfs.land(targetHeight=0.06, duration=2.0)
            timeHelper.sleep(3)
            # Disarm, if armed above (Crazyflie 2.1 Brushless and Bolt-based drones).
            # allcfs.arm(False)
            return 1

        # now that battery is low, we try to land on the pad and see if it's charging
        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(5)
        status = allcfs.crazyflies[0].get_status()

        # if not charging, take off and land back again until it charges
        while status['pm_state'] != 1:
            print('Not charging, retrying')
            allcfs.takeoff(targetHeight=1.0, duration=2.0)
            timeHelper.sleep(2.5)
            for cf in allcfs.crazyflies:
                pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
            timeHelper.sleep(2.5)
            allcfs.land(targetHeight=0.06, duration=2.0)
            timeHelper.sleep(5)
            status = allcfs.crazyflies[0].get_status()

        # Disarm while charging, if armed above (Crazyflie 2.1 Brushless and Bolt-based drones).
        # allcfs.arm(False)

        # now we wait until the crazyflie is charged
        # while status['battery'] < 4.1:
        while status['pm_state'] != 2:
            print(f'Charging in progress, battery at {status["battery"]}V')
            timeHelper.sleep(60)
            status = allcfs.crazyflies[0].get_status()
            # check if it's still charging ###not sure if this check is useful
            if status['pm_state'] != 1:
                print(f'charging interrupted, pm state : {status["pm_state"]}')

        print('Charging finished, time to fly again')
        flight_counter += 1


if __name__ == '__main__':
    main()
