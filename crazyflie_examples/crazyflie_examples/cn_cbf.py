#!/usr/bin/env python

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from crazyflie_interfaces.msg import ObstacleArray
from std_msgs.msg import Float32
import os
import casadi as ca
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor
from ament_index_python.packages import get_package_share_directory


class Sin(nn.Module):
    """Sine activation function"""

    def __init__(self):
        super().__init__()

    def forward(self, input: Tensor) -> Tensor:
        return torch.sin(input)


def get_func(func_name: str):
    """Returns a torch function based on the 'func_name' argument."""

    func_bank = {
        "linear": nn.Identity(),
        "relu": nn.ReLU(),
        "elu": nn.ELU(),
        "selu": nn.SELU(),
        "softplus": nn.Softplus(),
        "sigmoid": nn.Sigmoid(),
        "tanh": nn.Tanh(),
        "sin": Sin(),
    }
    if func_name not in func_bank.keys():
        raise KeyError(f"Invalid function name: '{func_name}'")
    return func_bank[func_name]


class MLP(nn.Module):
    """Multi-Layer Perceptron (MLP) model class."""

    def __init__(self, config: dict) -> None:
        super().__init__()
        self.model = nn.Sequential()
        for i in range(len(config["layers"])):
            in_features = config["input_size"] if i == 0 else config["layers"][i - 1][0]
            out_features = config["layers"][i][0]
            self.model.add_module(f"layer_{i}", nn.Linear(in_features, out_features))
            self.model.add_module(f"activation_{i}", get_func(config["layers"][i][1]))

    def forward(self, input: Tensor) -> Tensor:
        return self.model(input)

    def num_params(self) -> int:
        return sum(p.numel() for p in self.model.parameters())


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


def softmin_numpy(x, beta=10.0, axis=None):
    """Numpy implementation of the softmin function."""
    out = -(1.0 / beta) * np.log(np.sum(np.exp(-beta * x), axis=axis, keepdims=True))
    return out.squeeze()


def softmin_torch(x, beta=10.0, dim=None):
    """PyTorch implementation of the softmin function."""
    return -torch.logsumexp(-beta * x, dim=dim) / beta


class CN_CBF:
    """Composite Neural Control Barrier Function (CN-CBF) class."""

    def __init__(self):
        # Configuration parameters
        self.alpha = 4.0
        self.beta = 4.0
        self.r_min = 0.3
        self.mlp_device = "cpu"
        self.mlp_weights_filename = (
            # "cn_cbf_2d_double_integrator_v1.pth"  # v1-r_min=0.7;
            "cn_cbf_quadcopter_v5.pth" #r_min 0.3
            # "cn_cbf_quadcopter_v7.pth"
        )
        self.mlp_config = {
            "input_size": 4,
            "layers": [
                [48, "sin"],
                [48, "sin"],
                [24, "sin"],
                [24, "sin"],
                [12, "sin"],
                [12, "sin"],
                [1, "softplus"],
            ],
        }

        # Define system dynamics
        self.system_dynamics = DoubleIntegrator2D()

        # Define the MLP model and load weights
        pkg_share_dir = get_package_share_directory("crazyflie_examples")
        self.mlp = MLP(config=self.mlp_config).to(self.mlp_device)
        mlp_weights_path = os.path.join(
            pkg_share_dir, "data", "model_weights", self.mlp_weights_filename
        )
        self.mlp.load_state_dict(
            torch.load(
                f=mlp_weights_path,
                map_location=self.mlp_device,
                weights_only=True,
            )
        )

        # Define CBF-QP and create QP solver
        system_state = ca.MX.sym("system_state", self.system_dynamics.nx)
        nom_control = ca.MX.sym("nom_control", self.system_dynamics.nu)
        control = ca.MX.sym("control", self.system_dynamics.nu)
        cbf_value = ca.MX.sym("cbf_value")
        cbf_dt = ca.MX.sym("cbf_dt")
        cbf_grad = ca.MX.sym("cbf_grad", self.system_dynamics.nx)
        slack_var = ca.MX.sym("slack_var")

        # QP variables
        qp_var = ca.vertcat(control, slack_var)

        # QP objective
        cbf_obj = ca.dot((nom_control - control), (nom_control - control))
        qp_obj = cbf_obj + 1e6 * slack_var**2

        # QP constraints
        f = self.system_dynamics.open_loop_dynamics(system_state)
        g = self.system_dynamics.control_jacobian(system_state)
        cbf_constr = cbf_grad.T @ (f + g @ control) + cbf_dt + self.alpha * cbf_value
        qp_constr = cbf_constr + slack_var

        # QP parameters
        qp_params = ca.vertcat(
            system_state,
            nom_control,
            cbf_value,
            cbf_dt,
            cbf_grad,
        )

        # QP options
        qp_opts = {
            "error_on_fail": False,
        }

        qp = {"x": qp_var, "f": qp_obj, "g": qp_constr, "p": qp_params}

        self.qp_solver = ca.qpsol("qp_solver", "qpoases", qp, qp_opts)

        # Initialize variables
        self.obstacles_states = np.array([])
        self.obstacles_states_dot = np.array([])
        self.robot_state = np.zeros(self.system_dynamics.nx)
        self.nom_control = np.zeros(self.system_dynamics.nu)

    def obstacles_callback(self, msg):
        obstacles_states = []
        obstacles_states_dot = []
        for obstacle in msg.obstacles:
            obstacle_state = np.array(
                [
                    obstacle.pose.position.x,
                    obstacle.pose.position.y,
                    obstacle.twist.linear.x,
                    obstacle.twist.linear.y,
                ]
            )
            obstacle_state_dot = np.array(
                [
                    obstacle.twist.linear.x,
                    obstacle.twist.linear.y,
                    obstacle.accel.x,
                    obstacle.accel.y,
                ]
            )

            obstacles_states.append(obstacle_state)
            obstacles_states_dot.append(obstacle_state_dot)

        self.obstacles_states = np.array(obstacles_states)
        self.obstacles_states_dot = np.array(obstacles_states_dot)

    def cbf_qp_callback(self, robot_state, nom_control):
        # Solve the CBF-QP to get safe control
        if len(self.obstacles_states) > 0:
            robot_state = torch.tensor(
                data=robot_state,
                dtype=torch.float,
                device=self.mlp_device,
                requires_grad=True,
            )
            obstacle_states = torch.tensor(
                data=self.obstacles_states,
                dtype=torch.float,
                device=self.mlp_device,
                requires_grad=True,
            )
            obstacle_states_dot = torch.tensor(
                data=self.obstacles_states_dot,
                dtype=torch.float,
                device=self.mlp_device,
                requires_grad=False,
            )

            relative_states = obstacle_states - robot_state

            sdf_value = relative_states[:, :2].norm(dim=-1) - self.r_min
            n_cbf_value = sdf_value - self.mlp(relative_states).squeeze()

            cn_cbf_value = softmin_torch(
                n_cbf_value,
                beta=self.beta,
                dim=0,
            )
            cn_cbf_value.backward()
            cn_cbf_dt = (obstacle_states.grad * obstacle_states_dot).sum()
            cn_cbf_grad = robot_state.grad

            qp_params = ca.vertcat(
                ca.DM(robot_state.detach().numpy()),
                ca.DM(nom_control),
                ca.DM(cn_cbf_value.detach().numpy()),
                ca.DM(cn_cbf_dt.detach().numpy()),
                ca.DM(cn_cbf_grad.detach().numpy()),
            )
            opt_sol = self.qp_solver(
                x0=ca.vertcat(ca.DM(nom_control), 0.0),
                lbx=self.system_dynamics.u_min + [0.0],
                ubx=self.system_dynamics.u_max + [ca.inf],
                lbg=0.0,
                ubg=ca.inf,
                p=qp_params,
            )

            safe_control = opt_sol["x"].full().flatten()[: self.system_dynamics.nu]

            # TODO: Create publisher to publish CBF value
            # cbf_value_msg = Float32()
            # cbf_value_msg.data = cn_cbf_value.detach().item()
            # self.cbf_value_pub.publish(cbf_value_msg)
        else:
            safe_control = np.clip(
                nom_control, self.system_dynamics.u_min, self.system_dynamics.u_max
            )

        return safe_control


def executeTrajectory(
    timeHelper,
    cf,
    max_duration: float,
    rate: float = 100.0,
    pos_offset: np.ndarray = np.zeros(3),
    x_ref: np.ndarray = np.zeros([4]),
    K_lqr: np.ndarray = np.zeros([2, 4]),
):
    cn_cbf = CN_CBF()

    dt_nom = 1.0 / float(rate)

    start_time = timeHelper.time()
    last_time = start_time
    duration = 0.0

    # Initialize CF full state
    pos = np.array([0.0, -2.0, 0.0], dtype=float)
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
            cn_cbf.obstacles_callback(obstacles_msg)

        # Based on the current state and reference, compute the nominal control using LQR
        x = np.concatenate([pos[:2], vel[:2]], axis=0)
        acc_nom = -K_lqr @ (x - x_ref)

        # Use the CN-CBF to compute the safe control
        acc_safe = cn_cbf.cbf_qp_callback(x, acc_nom)

        # Update CF full state
        acc[:2] = acc_safe
        vel = np.clip(vel + acc * dt, -1.0, 1.0)
        print(vel)
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
