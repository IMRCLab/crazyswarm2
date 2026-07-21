from pathlib import Path

import numpy as np
import pinocchio as pin
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State


class Backend:
    """Backend that uses pinocchio to simulate the rigid-body dynamics."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.0005

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        next_states = []

        for uav, action in zip(self.uavs, actions):
            uav.step(action, self.dt)
            next_states.append(uav.state)

        # print(states_desired, actions, next_states)
        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


# convert RPM -> Force
def rpm_to_force(rpm):
    # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
    p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
    force_in_grams = np.polyval(p, rpm)
    force_in_newton = force_in_grams * 9.81 / 1000.0
    return np.maximum(force_in_newton, 0)


def pinocchio_state2sim_state(q, v):
    result = State()
    result.pos = q[0:3]
    result.quat = np.concatenate((q[6:7], q[3:6]))
    result.vel = v[0:3]
    result.omega = v[3:6]
    return result


def sim_state2pinocchio_state(state):
    result = np.empty(13)
    result[0:3] = state.pos
    result[3:6] = state.quat[1:]
    result[6] = state.quat[0]
    result[7:10] = state.vel
    result[10:13] = state.omega
    q = result[0:7]  # geometric states: x, y, z, qx, qy, qz, qw
    v = result[7:]  # velocities: vx, vy, vz, wx, wy, wz
    return q, v


class Quadrotor:
    """Quadrotor model using pinocchio."""

    def __init__(self, state):
        arm_length = 0.046  # m
        arm = 0.707106781 * arm_length
        t2t = 0.006  # thrust-to-torque ratio
        self.B0 = np.array([
            [1, 1, 1, 1],
            [-arm, -arm, arm, arm],
            [-arm, arm, arm, -arm],
            [-t2t, t2t, -t2t, t2t]
            ])
        self.uav, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            str((Path(__file__).parent / 'data/pinocchio/crazyflie2.urdf')))
        self.uavPinocchioData = self.uav.createData()
        self.state = state

    def step(self, action, dt):

        # m: 0.034
        # max_f: 1.3
        force_in_newton = rpm_to_force(action.rpm)
        eta = np.dot(self.B0, force_in_newton)
        force_generalized = np.array([0., 0., eta[0], eta[1], eta[2], eta[3]])

        q, v = sim_state2pinocchio_state(self.state)
        a = pin.aba(self.uav, self.uavPinocchioData, q, v, force_generalized)
        v_next = v + a*dt
        q_next = pin.integrate(self.uav, q, v*dt)
        self.state = pinocchio_state2sim_state(q_next, v_next)

        # if we fall below the ground, set velocities to 0
        if self.state.pos[2] < 0:
            self.state.pos[2] = 0
            self.state.vel = [0, 0, 0]
            self.state.omega = [0, 0, 0]
