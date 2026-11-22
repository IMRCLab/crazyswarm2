from __future__ import annotations

from math import cos, sin, sqrt

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State


class Backend:
    """Backend that uses newton-euler rigid-body dynamics implemented in numpy."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.0005
        # /clock is what use_sim_time nodes read. Publishing it on every 0.5 ms
        # step means 2000 messages per simulated second, which costs more than
        # the physics it reports. 200 Hz is finer than any control or logging
        # loop needs; set clock_rate to 0 in server.yaml for every step.
        try:
            rate = node._ros_parameters['sim'].get('clock_rate', 200.0)
        except (AttributeError, KeyError, TypeError):
            rate = 200.0
        self.clock_period = 1.0 / rate if rate and rate > 0 else 0.0
        self.clock_next_t = 0.0

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
        if self.clock_period <= 0.0 or self.t >= self.clock_next_t:
            self.clock_next_t = self.t + self.clock_period
            clock_message = Clock()
            clock_message.clock = Time(seconds=self.time()).to_msg()
            self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


class Quadrotor:
    """Basic rigid body quadrotor model (no drag)."""

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
        self.arm_length = 0.046  # m
        arm = 0.707106781 * self.arm_length
        self.t2t = t2t = 0.006  # thrust-to-torque ratio
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
        # This runs once per drone per 0.5 ms of simulated time, so at real-time
        # speed it is called 2000 times a second per drone. Writing it out in
        # scalar arithmetic rather than as rowan/numpy calls on 3- and 4-vectors
        # avoids the per-call dispatch overhead, which dominated the runtime at
        # this array size. The formulas are unchanged -- see
        # test/test_backend_np.py, which checks this against a direct
        # transcription of the previous implementation.
        rpm = action.rpm

        # convert RPM -> Force
        # polyfit using data and scripts from
        # https://github.com/IMRCLab/crazyflie-system-id
        newton_per_gram = 9.81 / 1000.0
        f = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            r = rpm[i]
            grams = (2.55077341e-08 * r - 4.92422570e-05) * r - 1.51910248e-01
            f[i] = grams * newton_per_gram if grams > 0.0 else 0.0

        # eta = B0 @ force, with B0 written out
        arm = 0.707106781 * self.arm_length
        thrust = f[0] + f[1] + f[2] + f[3]
        tau_x = arm * (-f[0] - f[1] + f[2] + f[3])
        tau_y = arm * (-f[0] + f[1] + f[2] - f[3])
        tau_z = self.t2t * (-f[0] + f[1] - f[2] + f[3])

        st = self.state._state
        px, py, pz = st[0], st[1], st[2]
        vx, vy, vz = st[3], st[4], st[5]
        qw, qx, qy, qz = st[6], st[7], st[8], st[9]
        wx, wy, wz = st[10], st[11], st[12]

        # dot{p} = v
        px += vx * dt
        py += vy * dt
        pz += vz * dt

        # mv = mg + R f_u + f_a, where f_u = [0, 0, thrust], so R f_u only needs
        # the third column of the rotation matrix
        r02 = 2.0 * (qx * qz + qw * qy)
        r12 = 2.0 * (qy * qz - qw * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        inv_m = 1.0 / self.mass
        vx += (r02 * thrust + f_a[0]) * inv_m * dt
        vy += (r12 * thrust + f_a[1]) * inv_m * dt
        vz += (-self.g + (r22 * thrust + f_a[2]) * inv_m) * dt

        # omega_global = R omega
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        gx = r00 * wx + r01 * wy + r02 * wz
        gy = r10 * wx + r11 * wy + r12 * wz
        gz = r20 * wx + r21 * wy + r22 * wz

        # dot{R} = R S(w): integrate the quaternion over the global angular
        # velocity, then renormalize. Same exponential map rowan.calculus.
        # integrate uses; see
        # https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/, and
        # Sec 4.5, https://arxiv.org/pdf/1711.02508.pdf
        hx, hy, hz = gx * dt * 0.5, gy * dt * 0.5, gz * dt * 0.5
        theta = sqrt(hx * hx + hy * hy + hz * hz)
        if theta > 0.0:
            scale = sin(theta) / theta
            ew, ex, ey, ez = cos(theta), scale * hx, scale * hy, scale * hz
        else:
            ew, ex, ey, ez = 1.0, 0.0, 0.0, 0.0
        nw = ew * qw - ex * qx - ey * qy - ez * qz
        nx = ew * qx + ex * qw + ey * qz - ez * qy
        ny = ew * qy - ex * qz + ey * qw + ez * qx
        nz = ew * qz + ex * qy - ey * qx + ez * qw
        inv_n = 1.0 / sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
        nw, nx, ny, nz = nw * inv_n, nx * inv_n, ny * inv_n, nz * inv_n

        # mJ = Jw x w + tau_u, with J diagonal
        jx, jy, jz = self.J[0], self.J[1], self.J[2]
        cx = (jy * wy) * wz - (jz * wz) * wy
        cy = (jz * wz) * wx - (jx * wx) * wz
        cz = (jx * wx) * wy - (jy * wy) * wx
        wx += (cx + tau_x) / jx * dt
        wy += (cy + tau_y) / jy * dt
        wz += (cz + tau_z) / jz * dt

        # if we fall below the ground, set velocities to 0
        if pz < 0.0:
            pz = 0.0
            vx, vy, vz = 0.0, 0.0, 0.0
            wx, wy, wz = 0.0, 0.0, 0.0

        st[0], st[1], st[2] = px, py, pz
        st[3], st[4], st[5] = vx, vy, vz
        st[6], st[7], st[8], st[9] = nw, nx, ny, nz
        st[10], st[11], st[12] = wx, wy, wz
