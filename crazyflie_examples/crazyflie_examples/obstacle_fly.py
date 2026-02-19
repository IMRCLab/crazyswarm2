#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    cfs = ["cf5"]
    TIMESCALE = 1.0

    allcfs = swarm.allcfs
    trajs = []

    for idx, cf in enumerate(cfs):
        traj = Trajectory()
        traj.loadcsv(Path(__file__).parent / f'data/obstacle_fly/traj{idx}.csv')
        trajs.append(traj)

    for idx, cf in enumerate(cfs):
        swarm.crazyfliesByName[cf].uploadTrajectory(0, 0, trajs[idx % len(trajs)])

    for cf in cfs:
        swarm.crazyfliesByName[cf].takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(3.0)
    
    for cf in cfs:
        pos = np.array(swarm.crazyfliesByName[cf].initialPosition) + np.array([0.0, 0.0, 1.0])
        swarm.crazyfliesByName[cf].goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    for cf in cfs:
        swarm.crazyfliesByName[cf].startTrajectory(0, TIMESCALE)
    timeHelper.sleep(max([t.duration for t in trajs]) * TIMESCALE + 2.0)

    for cf in cfs:
        swarm.crazyfliesByName[cf].land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)



if __name__ == '__main__':
    main()
