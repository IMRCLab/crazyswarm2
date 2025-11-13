from __future__ import annotations

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State


# Quaternion utility functions (replacing rowan dependency)
# NOTE: Quaternion format throughout is [qw, qx, qy, qz] (scalar-first),
# matching sim_data_types.State documentation.

def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize a quaternion [qw, qx, qy, qz]."""
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of quaternion [qw, qx, qy, qz]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 ⊗ q2 for [qw, qx, qy, qz] ordering."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v (3,) by quaternion q=[qw,qx,qy,qz] using v' = q ⊗ [0,v] ⊗ q*.
    Assumes q maps body -> world if used as in this simulator.
    """
    qn = quat_normalize(q)
    vq = np.array([0.0, v[0], v[1], v[2]])
    return quat_multiply(quat_multiply(qn, vq), quat_conjugate(qn))[1:4]


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Quaternion [qw,qx,qy,qz] from axis (3,) and angle (rad)."""
    axis_n = axis / (np.linalg.norm(axis) + 1e-16)
    half = angle * 0.5
    s = np.sin(half)
    c = np.cos(half)
    return np.array([c, axis_n[0]*s, axis_n[1]*s, axis_n[2]*s])


def quat_integrate(q: np.ndarray, omega_world: np.ndarray, dt: float) -> np.ndarray:
    """Integrate quaternion with world-frame angular velocity over dt.
    Uses exponential map: q_next = exp(0.5*dt*Omega) ⊗ q, where exp maps so(3)->SU(2).
    """
    theta = np.linalg.norm(omega_world) * dt
    if theta < 1e-12:
        return q
    dq = quat_from_axis_angle(omega_world, theta)
    # dq corresponds to rotation by angle theta about axis omega_world/||omega|| in world frame
    # For world-frame omega, apply left-multiplication
    return quat_multiply(dq, q)


class Backend:
    """Backend that uses newton-euler rigid-body dynamics implemented in numpy."""

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


class Quadrotor:
    """Basic rigid body quadrotor model (no drag) using numpy."""

    def __init__(self, state):
        # parameters (Crazyflie 2.0 quadrotor)
        self.mass = 0.034  # kg
        # self.J = np.array([
        # 	[16.56,0.83,0.71],
        # 	[0.83,16.66,1.8],
        # 	[0.72,1.8,29.26]
        # 	]) * 1e-6  # kg m^2
        self.J = np.array([16.571710e-6, 16.655602e-6, 29.261652e-6])

        # Note: we assume here that our control is forces
        arm_length = 0.046  # m
        arm = 0.707106781 * arm_length
        t2t = 0.006  # thrust-to-torque ratio
        self.B0 = np.array([
            [1, 1, 1, 1],
            [-arm, -arm, arm, arm],
            [-arm, arm, arm, -arm],
            [-t2t, t2t, -t2t, t2t]
            ])
        self.g = 9.81  # not signed

        if self.J.shape == (3, 3):
            self.inv_J = np.linalg.pinv(self.J)  # full matrix -> pseudo inverse
        else:
            self.inv_J = 1 / self.J  # diagonal matrix -> division

        self.state = state

    def step(self, action, dt, f_a=np.zeros(3)):

        # convert RPM -> Force
        def rpm_to_force(rpm):
            # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
            p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
            force_in_grams = np.polyval(p, rpm)
            force_in_newton = force_in_grams * 9.81 / 1000.0
            return np.maximum(force_in_newton, 0)

        force = rpm_to_force(action.rpm)

        # compute next state
        eta = np.dot(self.B0, force)
        f_u = np.array([0, 0, eta[0]])
        tau_u = np.array([eta[1], eta[2], eta[3]])

        # dynamics
        # dot{p} = v
        pos_next = self.state.pos + self.state.vel * dt
        # mv = mg + R f_u + f_a
        vel_next = self.state.vel + (
            np.array([0, 0, -self.g]) +
            (quat_rotate(self.state.quat, f_u) + f_a) / self.mass) * dt

        # dot{R} = R S(w)
        # to integrate the dynamics, see
        # https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/, and
        # https://arxiv.org/pdf/1604.08139.pdf
        # Sec 4.5, https://arxiv.org/pdf/1711.02508.pdf
        omega_global = quat_rotate(self.state.quat, self.state.omega)
        q_next = quat_normalize(
            quat_integrate(
                self.state.quat, omega_global, dt))

        # mJ = Jw x w + tau_u
        omega_next = self.state.omega + (
            self.inv_J * (np.cross(self.J * self.state.omega, self.state.omega) + tau_u)) * dt

        self.state.pos = pos_next
        self.state.vel = vel_next
        self.state.quat = q_next
        self.state.omega = omega_next

        # if we fall below the ground, set velocities to 0
        if self.state.pos[2] < 0:
            self.state.pos[2] = 0
            self.state.vel = [0, 0, 0]
            self.state.omega = [0, 0, 0]
