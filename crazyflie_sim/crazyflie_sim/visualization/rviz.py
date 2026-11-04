from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ..sim_data_types import Action, State


class Visualization:
    """Publishes ROS 2 transforms of the states, so that they can be visualized in RVIZ."""

    def __init__(
        self,
        node: Node,
        params: dict,
        names: list[str],
        states: list[State],
        reference_frames: list[str],
    ):
        self.node = node
        self.names = names
        self.reference_frames = reference_frames
        self.tfbr = TransformBroadcaster(self.node)
        # The physics loop steps every 0.5 ms, so broadcasting on every step puts
        # thousands of transforms a second on /tf. Nothing consuming them (rviz2,
        # the gui) redraws anywhere near that fast, and the publishing itself is
        # a measurable part of the sim's runtime. Set rate to 0 in server.yaml to
        # go back to broadcasting on every step.
        rate = params.get('rate', 100.0) if params else 100.0
        self.tf_period = 1.0 / rate if rate and rate > 0 else 0.0
        self.tf_next_t = 0.0

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        # publish transformation to visualize in rviz
        if self.tf_period > 0.0:
            if t < self.tf_next_t:
                return
            self.tf_next_t = t + self.tf_period
        msgs = []
        for name, state, reference_frame in zip(self.names, states, self.reference_frames):
            msg = TransformStamped()
            msg.header.stamp.sec = math.floor(t)
            msg.header.stamp.nanosec = int((t - msg.header.stamp.sec) * 1e9)
            msg.header.frame_id = reference_frame
            msg.child_frame_id = name
            msg.transform.translation.x = state.pos[0]
            msg.transform.translation.y = state.pos[1]
            msg.transform.translation.z = state.pos[2]
            msg.transform.rotation.x = state.quat[1]
            msg.transform.rotation.y = state.quat[2]
            msg.transform.rotation.z = state.quat[3]
            msg.transform.rotation.w = state.quat[0]
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

    def shutdown(self):
        pass
