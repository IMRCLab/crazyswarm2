"""Check the scalar rigid-body integrator against the original implementation.

backend/np.py's Quadrotor.step was rewritten in scalar arithmetic for speed. The
reference below is a direct transcription of the rowan/numpy version it
replaced; the two must agree to floating-point round-off on arbitrary states.
"""

import numpy as np
from crazyflie_sim.backend.np import Quadrotor
from crazyflie_sim.sim_data_types import Action, State
import rowan


def reference_step(uav, action, dt, f_a=np.zeros(3)):
    """Integrate one step the way the pre-optimization implementation did."""
    def rpm_to_force(rpm):
        p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
        force_in_grams = np.polyval(p, rpm)
        force_in_newton = force_in_grams * 9.81 / 1000.0
        return np.maximum(force_in_newton, 0)

    force = rpm_to_force(action.rpm)
    eta = np.dot(uav.B0, force)
    f_u = np.array([0, 0, eta[0]])
    tau_u = np.array([eta[1], eta[2], eta[3]])

    pos_next = uav.state.pos + uav.state.vel * dt
    vel_next = uav.state.vel + (
        np.array([0, 0, -uav.g])
        + (rowan.rotate(uav.state.quat, f_u) + f_a) / uav.mass) * dt
    omega_global = rowan.rotate(uav.state.quat, uav.state.omega)
    q_next = rowan.normalize(
        rowan.calculus.integrate(uav.state.quat, omega_global, dt))
    omega_next = uav.state.omega + (
        uav.inv_J * (np.cross(uav.J * uav.state.omega, uav.state.omega)
                     + tau_u)) * dt

    uav.state.pos = pos_next
    uav.state.vel = vel_next
    uav.state.quat = q_next
    uav.state.omega = omega_next
    if uav.state.pos[2] < 0:
        uav.state.pos[2] = 0
        uav.state.vel = [0, 0, 0]
        uav.state.omega = [0, 0, 0]


def random_state(rng):
    quat = rng.normal(size=4)
    quat /= np.linalg.norm(quat)
    return State(
        pos=rng.uniform(-2.0, 2.0, 3),
        vel=rng.uniform(-3.0, 3.0, 3),
        quat=quat,
        omega=rng.uniform(-10.0, 10.0, 3),
    )


def test_step_matches_reference():
    rng = np.random.default_rng(0)
    dt = 0.0005
    worst = 0.0
    for _ in range(500):
        state = random_state(rng)
        # hover RPM is about 12000; span idle through saturated
        action = Action(rng.uniform(0.0, 22000.0, 4))
        f_a = rng.uniform(-0.01, 0.01, 3)

        fast = Quadrotor(State(state.pos.copy(), state.vel.copy(),
                               state.quat.copy(), state.omega.copy()))
        ref = Quadrotor(State(state.pos.copy(), state.vel.copy(),
                              state.quat.copy(), state.omega.copy()))
        fast.step(action, dt, f_a)
        reference_step(ref, action, dt, f_a)

        # a quaternion and its negation are the same rotation
        if np.dot(fast.state.quat, ref.state.quat) < 0:
            ref.state.quat = -ref.state.quat
        worst = max(worst, float(
            np.abs(fast.state._state - ref.state._state).max()))
    assert worst < 1e-12, f'max state difference {worst:.3e}'


def test_step_integrates_a_hover():
    """Sanity check: thrust that cancels gravity should hold the drone up."""
    uav = Quadrotor(State())
    hover_rpm = 0.0
    lo, hi = 0.0, 30000.0
    for _ in range(60):                       # bisect for the hover RPM
        hover_rpm = 0.5 * (lo + hi)
        probe = Quadrotor(State())
        probe.step(Action(np.full(4, hover_rpm)), 0.001)
        if probe.state.vel[2] > 0:
            hi = hover_rpm
        else:
            lo = hover_rpm
    for _ in range(2000):
        uav.step(Action(np.full(4, hover_rpm)), 0.001)
    assert abs(uav.state.pos[2]) < 1e-3
    assert abs(uav.state.vel[2]) < 1e-3
