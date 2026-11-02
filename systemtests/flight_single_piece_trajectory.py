"""Exercise a one-row trajectory through the simulated server and /tf output."""

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from tf2_msgs.msg import TFMessage


def main():
    swarm = Crazyswarm()
    node = swarm.allcfs
    time_helper = swarm.timeHelper
    if not node.get_parameter('use_sim_time').value:
        raise RuntimeError('This regression must only run with simulated time')
    if not node.crazyflies:
        raise RuntimeError('No simulated Crazyflie configured')
    cf = node.crazyflies[0]
    frame = cf.prefix.lstrip('/')
    samples = []

    def receive_transforms(message):
        for transform in message.transforms:
            if transform.child_frame_id.lstrip('/') == frame:
                position = transform.transform.translation
                stamp = transform.header.stamp
                samples.append((stamp.sec + stamp.nanosec / 1e9,
                                [position.x, position.y, position.z]))

    subscription = node.create_subscription(TFMessage, '/tf', receive_transforms, 10)
    trajectory = Trajectory()
    # Exactly one data row: loadcsv raised IndexError here before ndmin=2.
    trajectory.loadcsv(Path(__file__).parent / 'data/single_piece.csv')
    cf.uploadTrajectory(0, 0, trajectory)
    origin = np.asarray(cf.initialPosition) + [0.0, 0.0, 1.0]
    try:
        cf.takeoff(targetHeight=1.0, duration=2.0)
        time_helper.sleep(2.5)
        cf.goTo(origin, 0.0, 2.0)
        time_helper.sleep(2.5)
        if not samples:
            raise AssertionError('No simulator /tf feedback received')
        np.testing.assert_allclose(samples[-1][1], origin, atol=0.1, rtol=0)

        samples.clear()
        start = time_helper.time()
        cf.startTrajectory(0, timescale=1.0, relative=True)
        time_helper.sleep(trajectory.duration + 2.0)
        # Check observed flight, not internal polynomial storage or desired setpoints.
        settled = [position for stamp, position in samples
                   if stamp >= start + trajectory.duration + 1.0]
        if len(settled) < 2:
            raise AssertionError('Missing fresh /tf feedback after trajectory execution')
        expected = origin + [0.5, 0.0, 0.0]
        np.testing.assert_allclose(settled, np.tile(expected, (len(settled), 1)),
                                   atol=0.1, rtol=0)
    finally:
        cf.land(targetHeight=0.06, duration=2.0)
        time_helper.sleep(3.0)
        node.destroy_subscription(subscription)


if __name__ == '__main__':
    main()
