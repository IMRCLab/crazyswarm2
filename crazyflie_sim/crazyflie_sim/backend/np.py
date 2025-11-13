from __future__ import annotations

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State


# Quaternion utility functions (replacing rowan dependency)
def quat_rotate(q, v):
    """Rotate vector v by quaternion q. Quaternion format: [x, y, z, w]"""
    # Extract quaternion components
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    
    # Extract vector components
    vx, vy, vz = v[0], v[1], v[2]
    
    # Perform rotation: v' = q * v * q^(-1)
    # Using formula: v' = v + 2 * cross(q_vec, cross(q_vec, v) + q_w * v)
    qvec = np.array([qx, qy, qz])
    cross1 = np.cross(qvec, v)
    cross2 = cross1 + qw * v
    cross3 = np.cross(qvec, cross2)
    return v + 2.0 * cross3


def quat_normalize(q):
    """Normalize a quaternion. Quaternion format: [x, y, z, w]"""
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm


def quat_integrate(q, omega, dt):
    """Integrate quaternion with angular velocity omega over timestep dt.
    q: quaternion [x, y, z, w]
    omega: angular velocity in body frame [wx, wy, wz]
    dt: timestep
    """
    # Quaternion derivative: dq/dt = 0.5 * q * omega_quat
    # where omega_quat = [omega_x, omega_y, omega_z, 0]
    
    # For small dt, we can use: q(t+dt) ≈ q(t) + 0.5 * dt * q(t) * omega_quat
    # More accurately, we use exponential map for integration
    
    theta = np.linalg.norm(omega) * dt
    
    if theta < 1e-10:
        # No rotation
        return q
    
    # Axis of rotation (normalized)
    axis = omega / np.linalg.norm(omega)
    
    # Quaternion representing the rotation: [sin(theta/2)*axis, cos(theta/2)]
    half_theta = theta / 2.0
    sin_half = np.sin(half_theta)
    cos_half = np.cos(half_theta)
    
    dq = np.array([
        sin_half * axis[0],
        sin_half * axis[1],
        sin_half * axis[2],
        cos_half
    ])
    
    # Quaternion multiplication: q_new = dq * q
    return quat_multiply(dq, q)


def quat_multiply(q1, q2):
    """Multiply two quaternions. Quaternion format: [x, y, z, w]"""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


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
    """Basic rigid body quadrotor model (no drag) using numpy and rowan."""

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
