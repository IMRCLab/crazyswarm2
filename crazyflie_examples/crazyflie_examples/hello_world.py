"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # Crazyflie 2.1 Brushless (and Bolt-based drones) must be armed before flight.
    # Either arm using the joystick or uncomment the following lines.
    # cf.arm(True)
    # timeHelper.sleep(1.0)

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)

    # Disarm, if armed above (Crazyflie 2.1 Brushless and Bolt-based drones).
    # cf.arm(False)


if __name__ == '__main__':
    main()
