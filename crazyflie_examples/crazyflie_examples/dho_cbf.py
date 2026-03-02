#!/usr/bin/env python

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from crazyflie_interfaces.msg import ObstacleArray
import casadi as ca


class DoubleIntegrator2D:
    """2D Double Integrator model class."""

    def __init__(self):
        # State space
        self.nx = 4
        self.x_min = [-ca.inf, -ca.inf, -2.0, -2.0]
        self.x_max = [ca.inf, ca.inf, 2.0, 2.0]
        self.A = ca.DM(
            [
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
            ]
        )

        # Control space
        self.nu = 2
        self.u_min = [-2.0, -2.0]
        self.u_max = [2.0, 2.0]
        self.B = ca.DM(
            [
                [0.0, 0.0],
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
            ]
        )

    def open_loop_dynamics(self, x):
        return self.A @ x

    def control_jacobian(self, x):
        return self.B


class DHO_CBF:
    """Dynamic Higher-Order Control Barrier Function (DHO-CBF) class."""

    def __init__(self):
        # Configuration parameters
        self.k1 = 2.0
        self.k2 = 2.0
        self.max_num_obstacles = 5
        self.obstacle_state_size = 4  # [x, y, vx, vy]
        self.r_min = 0.3

        # Define system dynamics
        self.system_dynamics = DoubleIntegrator2D()

        # Define CBF-QP and create QP solver
        system_state = ca.MX.sym("system_state", self.system_dynamics.nx)
        nom_control = ca.MX.sym("nom_control", self.system_dynamics.nu)
        control = ca.MX.sym("control", self.system_dynamics.nu)
        obstacles_states = ca.MX.sym(
            "obstacles_states", self.max_num_obstacles, self.obstacle_state_size
        )
        obstacles_states_dot = ca.MX.sym(
            "obstacles_states_dot", self.max_num_obstacles, self.obstacle_state_size
        )
        slack_var = ca.MX.sym("slack_var", self.max_num_obstacles)

        # QP variables
        qp_var = ca.vertcat(control, slack_var)

        # QP objective
        qp_obj = ca.dot(
            (nom_control - control), (nom_control - control)
        ) + 1e6 * ca.dot(slack_var, slack_var)

        # QP constraints
        qp_constr = []
        for i in range(self.max_num_obstacles):
            p = obstacles_states[i, :2].T - system_state[:2]
            v = obstacles_states[i, 2:].T - system_state[2:]
            a = obstacles_states_dot[i, 2:].T - control

            h = ca.norm_2(p) - self.r_min
            h_dot = (p.T @ v) / ca.norm_2(p)
            h_ddot = (p.T @ a + v.T @ v) / ca.norm_2(p) - (p.T @ v) ** 2 / ca.norm_2(
                p
            ) ** 3

            cbf_constr = h_ddot + (self.k1 + self.k2) * h_dot + self.k1 * self.k2 * h
            qp_constr.append(cbf_constr + slack_var[i])
        qp_constr = ca.vertcat(*qp_constr)

        # QP parameters
        qp_params = ca.vertcat(
            system_state,
            nom_control,
            ca.vec(obstacles_states),
            ca.vec(obstacles_states_dot),
        )

        # QP options
        qp_opts = {
            "error_on_fail": False,
        }

        qp = {"x": qp_var, "f": qp_obj, "g": qp_constr, "p": qp_params}

        self.qp_solver = ca.qpsol("qp_solver", "qpoases", qp, qp_opts)

        # Initialize variables
        self.lbx = self.system_dynamics.u_min + [0.0] * self.max_num_obstacles
        self.ubx = self.system_dynamics.u_max + [ca.inf] * self.max_num_obstacles
        self.lbg = [0.0] * self.max_num_obstacles
        self.ubg = [ca.inf] * self.max_num_obstacles
        self.slack_0 = np.zeros(self.max_num_obstacles)
        self.robot_state = np.zeros(self.system_dynamics.nx)
        self.nom_control = np.zeros(self.system_dynamics.nu)

        # Initialize obstacles states properly to avoid numerical issues
        self.obstacles_states_init = np.zeros(
            (self.max_num_obstacles, self.obstacle_state_size)
        )
        self.obstacles_states_init[:, :2] = -100.0
        self.obstacles_states_init[:, 2] = -1.0
        self.obstacles_states = self.obstacles_states_init.copy()
        self.obstacles_states_dot = np.zeros(
            (self.max_num_obstacles, self.obstacle_state_size)
        )

    def obstacles_callback(self, msg):
        self.obstacles_states = self.obstacles_states_init.copy()
        self.obstacles_states_dot = np.zeros(
            (self.max_num_obstacles, self.obstacle_state_size)
        )
        for i in range(min(len(msg.obstacles), self.max_num_obstacles)):
            obstacle = msg.obstacles[i]
            self.obstacles_states[i, :] = np.array(
                [
                    obstacle.pose.position.x,
                    obstacle.pose.position.y,
                    obstacle.twist.linear.x,
                    obstacle.twist.linear.y,
                ]
            )
            self.obstacles_states_dot[i, :] = np.array(
                [
                    obstacle.twist.linear.x,
                    obstacle.twist.linear.y,
                    obstacle.accel.x,
                    obstacle.accel.y,
                ]
            )

    def cbf_qp_callback(self, robot_state, nom_control):
        # Solve the CBF-QP to get safe control
        qp_params = ca.vertcat(
            ca.DM(robot_state),
            ca.DM(nom_control),
            ca.vec(ca.DM(self.obstacles_states)),
            ca.vec(ca.DM(self.obstacles_states_dot)),
        )
        opt_sol = self.qp_solver(
            x0=ca.vertcat(ca.DM(nom_control), ca.DM(self.slack_0)),
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=qp_params,
        )
        return opt_sol["x"].full().flatten()[: self.system_dynamics.nu]


def executeTrajectory(
    timeHelper,
    cf,
    max_duration: float,
    rate: float = 100.0,
    pos_offset: np.ndarray = np.zeros(3),
    x_ref: np.ndarray = np.zeros([4]),
    K_lqr: np.ndarray = np.zeros([2, 4]),
):
    dho_cbf = DHO_CBF()

    dt_nom = 1.0 / float(rate)

    start_time = timeHelper.time()
    last_time = start_time
    duration = 0.0

    # Initialize CF full state
    pos = np.array([0.0, -2.5, 0.0], dtype=float)
    vel = np.array([0.0, 0.0, 0.0], dtype=float)
    acc = np.array([0.0, 0.0, 0.0], dtype=float)
    yaw = 0.0
    omega = np.zeros(3)

    while not timeHelper.isShutdown() and duration <= max_duration:
        now = timeHelper.time()
        dt = np.clip(now - last_time, 0.5 * dt_nom, 1.5 * dt_nom)
        last_time = now
        duration = now - start_time

        # Access latest obstacles msg
        obstacles_msg = getattr(timeHelper.node, "obstacles_msg", None)
        if obstacles_msg is not None:
            dho_cbf.obstacles_callback(obstacles_msg)

        # Based on the current state and reference, compute the nominal control using LQR
        x = np.concatenate([pos[:2], vel[:2]], axis=0)
        acc_nom = -K_lqr @ (x - x_ref)

        # Use the CN-CBF to compute the safe control
        acc_safe = dho_cbf.cbf_qp_callback(x, acc_nom)

        # Update CF full state
        acc[:2] = acc_safe
        vel = np.clip(vel + acc * dt, -1.0, 1.0)
        pos = pos + vel * dt

        cf.cmdFullState(
            pos + pos_offset,
            vel,
            acc,
            yaw,
            omega,
        )

        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyfliesByName["cf3"]

    def obstacles_callback(msg: ObstacleArray):
        setattr(timeHelper.node, "obstacles_msg", msg)
        print(f"Received {len(msg.obstacles)} obstacles.")

    timeHelper.node.create_subscription(
        ObstacleArray,
        "/obstacles",
        obstacles_callback,
        1,
    )

    # Give some time for the subscription to establish
    print("Waiting for obstacles data...")
    timeHelper.sleep(2.0)
    # while not hasattr(timeHelper.node, "obstacles_msg"):
    #     timeHelper.sleep(1.0)

    # high-level mode test
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "data/figure8.csv")
    cf.uploadTrajectory(0, 0, traj1)

    height_offset = 0.5  # 2D plane offset in z-axis
    x_ref = np.array([0, 2.5, 0, 0])  # reference state

    # K_lqr = np.array([[3.16, 0, 2.71, 0], [0, 3.16, 0, 2.71]])
    K_lqr = np.array([[2.0, 0, 2.236, 0], [0, 2.0, 0, 2.236]])  # Less aggressive gains

    cf.takeoff(targetHeight=height_offset, duration=height_offset + 1.0)
    timeHelper.sleep(height_offset + 2.0)

    executeTrajectory(
        timeHelper,
        cf,
        max_duration=15.0,  # in seconds
        rate=100.0,  # in Hz
        pos_offset=np.array([0, 0, height_offset]),
        x_ref=x_ref,
        K_lqr=K_lqr,
    )

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=height_offset + 1.0)
    timeHelper.sleep(height_offset + 2.0)


if __name__ == "__main__":
    main()
