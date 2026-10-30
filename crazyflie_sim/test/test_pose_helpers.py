"""Unit tests for simulated pose message construction."""

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
import rclpy

from crazyflie_sim.crazyflie_server import (
    CrazyflieServer,
    _pose_stamped_from_state,
    _simulation_time_to_stamp,
)
from crazyflie_sim.sim_data_types import State


def test_simulation_time_to_stamp_zero():
    sec, nanosec = _simulation_time_to_stamp(0.0)
    assert sec == 0
    assert nanosec == 0


def test_simulation_time_to_stamp_fractional():
    sec, nanosec = _simulation_time_to_stamp(1.5)
    assert sec == 1
    assert nanosec == 500000000


def test_simulation_time_to_stamp_matches_rviz_conversion():
    t = 12.3456789
    sec, nanosec = _simulation_time_to_stamp(t)
    assert sec == math.floor(t)
    assert nanosec == int((t - sec) * 1e9)


def test_pose_stamped_position_from_state():
    state = State(pos=np.array([1.25, -0.5, 2.0]))
    msg = _pose_stamped_from_state(state, 0.0, 'world')
    assert isinstance(msg, PoseStamped)
    assert msg.pose.position.x == 1.25
    assert msg.pose.position.y == -0.5
    assert msg.pose.position.z == 2.0


def test_pose_stamped_quaternion_qw_qx_qy_qz():
    # Identity would hide an x/y/z/w swap; use a non-identity quaternion.
    state = State(quat=np.array([0.5, 0.5, 0.5, 0.5]))
    msg = _pose_stamped_from_state(state, 0.0, 'world')
    assert msg.pose.orientation.w == 0.5
    assert msg.pose.orientation.x == 0.5
    assert msg.pose.orientation.y == 0.5
    assert msg.pose.orientation.z == 0.5


def test_pose_stamped_does_not_map_quat_sequentially_to_xyzw():
    state = State(quat=np.array([1.0, 0.0, 0.0, 0.0]))
    msg = _pose_stamped_from_state(state, 0.0, 'world')
    assert msg.pose.orientation.w == 1.0
    assert msg.pose.orientation.x == 0.0
    assert msg.pose.orientation.y == 0.0
    assert msg.pose.orientation.z == 0.0


def test_pose_stamped_quaternion_norm():
    state = State(quat=np.array([0.70710678118, 0.0, 0.70710678118, 0.0]))
    msg = _pose_stamped_from_state(state, 0.0, 'world')
    q = np.array([
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ])
    assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-6)


def test_pose_stamped_frame_id_and_timestamp():
    state = State(pos=np.array([0.0, 0.0, 1.0]))
    msg = _pose_stamped_from_state(state, 2.25, 'test_map')
    assert msg.header.frame_id == 'test_map'
    assert msg.header.stamp.sec == 2
    assert msg.header.stamp.nanosec == 250000000


def test_non_positive_pose_frequency_is_rejected():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', 'sim.pose_frequency:=0.0'])
    try:
        with pytest.raises(ValueError, match='pose_frequency'):
            CrazyflieServer()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_negative_pose_frequency_is_rejected():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', 'sim.pose_frequency:=-1.0'])
    try:
        with pytest.raises(ValueError, match='pose_frequency'):
            CrazyflieServer()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
