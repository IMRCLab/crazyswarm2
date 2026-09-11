"""Integration tests for simulated pose topics and get_position()."""

from contextlib import contextmanager
import threading
from tempfile import NamedTemporaryFile

from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml

from crazyflie_sim.crazyflie_server import CrazyflieServer


def _robot(name, initial_position, uri_suffix):
    return {
        name: {
            'enabled': True,
            'uri': 'radio://0/80/2M/E7E7E7E7{}'.format(uri_suffix),
            'initial_position': [float(v) for v in initial_position],
            'type': 'cf21',
        }
    }


def _sim_params(
        robots, backend='none', rviz_enabled=False, pose_frequency=10.0,
        controller='none', reference_frame='test_map'):
    return {
        'fileversion': 3,
        'robot_description': 'robot $NAME',
        'robots': robots,
        'robot_types': {
            'cf21': {
                'connection': 'crazyflie',
            }
        },
        'all': {
            'reference_frame': reference_frame,
        },
        'sim': {
            'max_dt': 0.0,
            'backend': backend,
            'controller': controller,
            'pose_frequency': pose_frequency,
            'visualizations': {
                'rviz': {
                    'enabled': rviz_enabled,
                }
            }
        }
    }


def _write_params_file(params):
    handle = NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(
        {'crazyflie_server': {'ros__parameters': params}}, handle)
    handle.close()
    return handle.name


class _PoseCollector(Node):
    """Collect PoseStamped messages for one or more robots."""

    def __init__(self, names):
        super().__init__('pose_collector')
        self.msgs = {name: [] for name in names}
        for name in names:
            self.create_subscription(
                PoseStamped,
                '{}/pose'.format(name),
                lambda msg, n=name: self.msgs[n].append(msg),
                10,
            )


@contextmanager
def _running_sim(params, collect_names=None):
    params_path = _write_params_file(params)
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=['--ros-args', '--params-file', params_path])
    server = CrazyflieServer()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    collector = None
    if collect_names:
        collector = _PoseCollector(collect_names)
        executor.add_node(collector)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield server, collector
    finally:
        executor.shutdown()
        if collector is not None:
            collector.destroy_node()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


def _wait_until(predicate, timeout_sec, spin_node=None):
    deadline = timeout_sec
    slept = 0.0
    step = 0.05
    while slept < deadline:
        if spin_node is not None:
            rclpy.spin_once(spin_node, timeout_sec=step)
        else:
            threading.Event().wait(step)
        if predicate():
            return True
        slept += step
    return predicate()


@pytest.mark.requires_firmware
def test_pose_messages_match_simulator_state():
    """Published PoseStamped messages come from simulator state."""
    robots = {}
    robots.update(_robot('cf1', [0.0, 0.0, 0.0], '01'))
    params = _sim_params(robots, backend='none', rviz_enabled=False)
    with _running_sim(params, collect_names=['cf1']) as (server, collector):
        assert server.visualizations == []
        ok = _wait_until(lambda: len(collector.msgs['cf1']) >= 1, 5.0)
        assert ok, 'no PoseStamped received on /cf1/pose'
        msg = collector.msgs['cf1'][-1]
        assert msg.header.frame_id == 'test_map'
        q = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-3)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        assert t >= 0.0
        assert abs(t - server.backend.time()) < 1.0


@pytest.mark.requires_firmware
def test_pose_publication_is_rate_limited():
    """Pose timestamps follow pose_frequency, not the physics rate."""
    robots = {}
    robots.update(_robot('cf1', [0.0, 0.0, 0.0], '01'))
    params = _sim_params(
        robots, backend='none', pose_frequency=5.0, rviz_enabled=False)
    with _running_sim(params, collect_names=['cf1']) as (_server, collector):
        ok = _wait_until(lambda: len(collector.msgs['cf1']) >= 4, 5.0)
        assert ok, 'not enough pose messages to check rate limiting'
        stamps = []
        for msg in collector.msgs['cf1']:
            stamps.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        dts = np.diff(stamps)
        # Physics dt is 0.1 s; a 5 Hz pose rate must not publish every step.
        assert np.all(dts >= 0.15)


@pytest.mark.requires_firmware
def test_get_position_after_takeoff_without_rviz():
    """Crazyflie.get_position() tracks simulated motion with RViz disabled."""
    pytest.importorskip('crazyflie_py')
    from crazyflie_py.crazyflie import CrazyflieServer as CrazyflieApi
    from crazyflie_py.crazyflie import TimeHelper

    robots = {}
    robots.update(_robot('cf1', [0.0, 0.0, 0.0], '01'))
    params = _sim_params(robots, backend='none', rviz_enabled=False)
    with _running_sim(params) as (server, _collector):
        assert server.visualizations == []
        api = CrazyflieApi()
        try:
            time_helper = TimeHelper(api)
            cf = api.crazyflies[0]
            cf.takeoff(targetHeight=1.0, duration=2.0)
            time_helper.sleep(2.5)
            position = np.asarray(cf.get_position())
            assert position[2] > 0.5
        finally:
            api.destroy_node()


@pytest.mark.requires_firmware
def test_multi_robot_pose_association():
    """Each Crazyflie object receives the corresponding vehicle state."""
    pytest.importorskip('crazyflie_py')
    pytest.importorskip('rowan')
    from crazyflie_py.crazyflie import CrazyflieServer as CrazyflieApi
    from crazyflie_py.crazyflie import TimeHelper

    robots = {}
    robots.update(_robot('cf1', [0.0, 0.0, 0.0], '01'))
    robots.update(_robot('cf2', [1.0, 0.0, 0.0], '02'))
    params = _sim_params(
        robots, backend='np', controller='mellinger', rviz_enabled=False)
    with _running_sim(params) as (_server, _collector):
        api = CrazyflieApi()
        try:
            time_helper = TimeHelper(api)
            by_name = api.crazyfliesByName
            assert 'cf1' in by_name and 'cf2' in by_name

            def _separated():
                p1 = np.asarray(by_name['cf1'].get_position())
                p2 = np.asarray(by_name['cf2'].get_position())
                return abs(p1[0]) < 0.3 and abs(p2[0] - 1.0) < 0.3

            ok = _wait_until(
                _separated, 5.0, spin_node=api)
            assert ok, 'robots did not receive distinct initial poses'
            p1 = np.asarray(by_name['cf1'].get_position())
            p2 = np.asarray(by_name['cf2'].get_position())
            assert not np.allclose(p1, p2)

            by_name['cf1'].takeoff(targetHeight=1.0, duration=2.0)
            by_name['cf2'].takeoff(targetHeight=0.5, duration=2.0)
            time_helper.sleep(2.5)
            p1 = np.asarray(by_name['cf1'].get_position())
            p2 = np.asarray(by_name['cf2'].get_position())
            assert p1[2] > 0.5
            assert p2[2] > 0.2
            assert abs(p1[0]) < 0.5
            assert abs(p2[0] - 1.0) < 0.5
        finally:
            api.destroy_node()
