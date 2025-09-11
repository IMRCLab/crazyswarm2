#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

import os
import casadi as ca
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor
from torch.nn.utils import vector_to_parameters
from scipy import ndimage
from ament_index_python.packages import get_package_share_directory


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3), yawrate=0.0):
    traj = Trajectory()
    traj.loadcsv(trajpath)

    start_time = timeHelper.time()
    yaw = 0
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break

        e = traj.eval(t)
        yaw = yaw + yawrate * 1/rate
        omega = e.omega
        omega[2] = yawrate
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            yaw,
            omega)
        print(yaw)

        timeHelper.sleepForRate(rate)

# def executeTrajectory2(timeHelper, cf, rate=100, offset=np.zeros(3), yawrate=0.0):

#     start_time = timeHelper.time()
#     yaw = 0
#     pos = np.zeros(3)
#     vel = np.zeros(3)
#     acc = np.array([0])
#     while not timeHelper.isShutdown():
#         t = timeHelper.time() - start_time
#         if t > 5.0:
#             break

#         e = traj.eval(t)
#         yaw = yaw + yawrate * 1/rate
#         omega = e.omega
#         omega[2] = yawrate
#         cf.cmdFullState(
#             e.pos + np.array(cf.initialPosition) + offset,
#             e.vel,
#             e.acc,
#             yaw,
#             omega)
#         print(yaw)

#         timeHelper.sleepForRate(rate)

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

class Hypernetwork(nn.Module):
    """Predicts parameters of the main network based on a 2D input."""

    def __init__(self, input_size: int, output_size: int) -> None:
        super().__init__()
        n = 8
        self.backbone = nn.Sequential(
            nn.Conv2d(input_size, n, (3, 3), (1, 1), "valid", bias=False),
            nn.SELU(),
            nn.Conv2d(n, n, (3, 3), (1, 1), "valid", bias=False),
            nn.SELU(),
            nn.AvgPool2d((2, 2), (1, 1)),
            nn.Conv2d(n, 2 * n, (3, 3), (2, 2), "valid", bias=False),
            # (2*n, 47, 47)
            nn.SELU(),
            nn.Conv2d(2 * n, 2 * n, (3, 3), (1, 1), "valid", bias=False),
            nn.SELU(),
            nn.AvgPool2d((2, 2), (1, 1)),
            nn.Conv2d(2 * n, 4 * n, (2, 2), (2, 2), "valid", bias=False),
            # (4*n, 22, 22)
            nn.SELU(),
            nn.Conv2d(4 * n, 4 * n, (3, 3), (2, 2), "valid", bias=False),
            nn.SELU(),
            nn.Conv2d(4 * n, 8 * n, (2, 2), (2, 2), "valid", bias=False),
            # (8*n, 4, 4)
            nn.SELU(),
            nn.Flatten(),
        )


        self.head = nn.Sequential(
            nn.Linear(1024, 1024),
            nn.SELU(),
            nn.Linear(1024, output_size),
        )

    def forward(self, input: Tensor) -> Tensor:
        input = self.backbone(input)
        return self.head(input)


class MainNetwork(nn.Module):
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
    """2D Double Integrator model of a robot."""
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
        self.u_min = [-1.0, -1.0]
        self.u_max = [1.0, 1.0]
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


class ORN_CBF:
    def __init__(self):
        self.alpha = 0.7
        self.sdf_size = 80
        self.sdf_patch_size = 16
        self.sdf_res = 0.075
        self.hypernet_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.hypernet_weights = "orn_cbf_2d_double_integrator.pth"
        self.main_net_device = "cpu"
        self.main_net_config = {
            "input_size": 4,
            "layers":
            [
                [32, "sin"],
                [32, "sin"],
                [32, "sin"],
                [16, "sin"],
                [16, "sin"],
                [16, "sin"],
                [8, "sin"],
                [8, "sin"],
                [8, "sin"],
                [1, "softplus"]
            ]
        }

        self.system_dynamics = DoubleIntegrator2D()

        self.main_net = MainNetwork(config=self.main_net_config).to(self.main_net_device)

        pkg_share_dir = get_package_share_directory("crazyflie_examples")
        hypernet_weights_path = os.path.join(
            pkg_share_dir, "crazyflie_examples", "data", self.hypernet_weights
        )

        self.hypernet = Hypernetwork(
            input_size=1,
            output_size=self.main_net.num_params(),
        ).to(self.hypernet_device)

        self.hypernet.load_state_dict(
            torch.load(
                f=hypernet_weights_path,
                map_location=self.hypernet_device,
                weights_only=True,
            )
        )

        # Define symbolic variables used by the CBF-QP
        system_state = ca.MX.sym("system_state", self.system_dynamics.nx)
        nom_control = ca.MX.sym("nom_control", self.system_dynamics.nu)
        control = ca.MX.sym("control", self.system_dynamics.nu)
        cbf_value = ca.MX.sym("cbf_value")
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
        cbf_constr = cbf_grad.T @ (f + g @ control) + self.alpha * cbf_value
        qp_constr = cbf_constr + slack_var

        # QP parameters
        qp_params = ca.vertcat(
            system_state,
            nom_control,
            cbf_value,
            cbf_grad,
        )

        # QP options
        qp_opts = {"error_on_fail": False}

        qp = {"x": qp_var, "f": qp_obj, "g": qp_constr, "p": qp_params}

        self.qp_solver = ca.qpsol("qp_solver", "qpoases", qp, qp_opts)

        # SDF patch related variables
        sdf_size = self.get_parameter("sdf_size").value
        self.sdf_patch_size = self.get_parameter("sdf_patch_size").value
        sdf_res = self.get_parameter("sdf_res").value
        self.patch_idx_min = (sdf_size - self.sdf_patch_size) // 2
        self.patch_idx_max = (sdf_size + self.sdf_patch_size) // 2
        self.sdf_patch_idx = [
            slice(None),
            slice(None),
            slice(self.patch_idx_min, self.patch_idx_max),
            slice(self.patch_idx_min, self.patch_idx_max),
        ]
        self.patch_scale_factor = (self.sdf_patch_size - 1) * sdf_res / 2

        # Initialize variables
        self.sdf_patch = torch.ones(
            (1, 1, self.sdf_patch_size, self.sdf_patch_size),
            device=self.main_net_device,
        )
        vector_to_parameters(
            torch.zeros(self.main_net.num_params(), device=self.main_net_device),
            self.main_net.parameters(),
        )

    def update_main_net(self, costmap_msg: OccupancyGrid):
        # Update the main neural network based on the latest costmap
        costmap = np.reshape(
            np.array(costmap_msg.data, dtype=np.bool_),
            (costmap_msg.info.size_y, costmap_msg.info.size_x),
        ).T

        # TODO: extract costmap position
        self.costmap_pos = np.zeros(2)

        if np.count_nonzero(costmap) > 0:
            sdf_pos = ndimage.distance_transform_edt(~costmap)
            sdf_neg = ndimage.distance_transform_edt(costmap)
            sdf = (sdf_pos - sdf_neg) * costmap_msg.info.resolution
            sdf = torch.tensor(
                data=sdf[None, None, ...],
                dtype=torch.float,
                device=self.hypernet_device,
            )

            main_net_params = self.hypernet(sdf).detach().to(self.main_net_device)

            vector_to_parameters(main_net_params.squeeze(), self.main_net.parameters())
            self.sdf_patch = (
                sdf[self.sdf_patch_idx].clone().detach().to(self.main_net_device)
            )
        else:
            vector_to_parameters(
                torch.zeros(self.main_net.num_params(), device=self.main_net_device),
                self.main_net.parameters(),
            )
            self.sdf_patch = torch.ones(
                (1, 1, self.sdf_patch_size, self.sdf_patch_size),
                device=self.main_net_device,
            )

    def cbf_qp(self, pos, vel, acc_nom):
        # Solve the CBF-QP to get safe acceleration
        rel_pos = pos - self.costmap_pos
        system_state = torch.tensor(
            data=np.concatenate((rel_pos, vel), axis=0),
            dtype=torch.float,
            device=self.main_net_device,
            requires_grad=True,
        )
        residual_value = self.main_net(system_state[None, ...])
        sdf_value = F.grid_sample(
            input=self.sdf_patch.transpose(-1, -2),
            grid=system_state[None, None, None, :2] / self.patch_scale_factor,
            mode="bilinear",
            align_corners=True,
            padding_mode="border",
        ).squeeze()

        cbf_value = sdf_value - residual_value
        cbf_value.backward()
        cbf_grad = system_state.grad

        qp_params = ca.vertcat(
            ca.DM(system_state.detach().numpy()),
            ca.DM(acc_nom),
            ca.DM(cbf_value.detach().numpy()),
            ca.DM(cbf_grad.detach().numpy()),
        )

        opt_sol = self.qp_solver(
            x0=ca.vertcat(ca.DM(self.nom_control), 10.0),
            lbx=self.system_dynamics.u_min + [0.0],
            ubx=self.system_dynamics.u_max + [ca.inf],
            lbg=0.0,
            ubg=ca.inf,
            p=qp_params,
        )

        return opt_sol["x"].full().flatten()[: self.system_dynamics.nu]


def executeTrajectory2(timeHelper, cf,
                        duration: float,
                        rate: float = 100.0,
                        offset: np.ndarray = np.zeros(3),
                        ay: float = 0.0,
                        v0: np.ndarray = np.array([0.0, 0.0, 0.0]),
                        yawrate: float = 0.0,    
                        spin_duration: float = 3.0,
                        spin_yawrate: float | None = None):

    vel = np.array(v0, dtype=float).reshape(3)
    pos = np.zeros(3, dtype=float)

    spin_yawrate = yawrate if spin_yawrate is None else float(spin_yawrate)

    orn_cbf = ORN_CBF()

    start_time = timeHelper.time()
    last_time = start_time
    yaw = float(0.0)
    omega = np.array([0.0, 0.0, yawrate], dtype=float)
    dt_nominal = 1.0 / float(rate)

    while not timeHelper.isShutdown():
        now = timeHelper.time()
        t = now - start_time
        if t > duration:
            break
        dt = now - last_time
        if dt <= 0: dt = dt_nominal
        elif dt > 2.0 * dt_nominal: dt = 2.0 * dt_nominal
        last_time = now

        # Access latest costmap if available on the node (set in main)
        latest_costmap = getattr(timeHelper.node, 'latest_costmap', None)
        
        # Print costmap info for debugging
        if latest_costmap is not None:
            print(f"Costmap available: resolution={latest_costmap.info.resolution}, "
                  f"width={latest_costmap.info.width}, height={latest_costmap.info.height}")
        else:
            print("No costmap available yet")

        if t < spin_duration:
            acc = np.zeros(3, dtype=float)
            omega = np.array([0.0, 0.0, spin_yawrate], dtype=float)
            yaw += spin_yawrate * dt
            cmd_pos = pos + np.array(cf.initialPosition, dtype=float) + offset
            cf.cmdFullState(cmd_pos, vel*0.0, acc, yaw, omega)

        else:
            # Placeholder: use observation (latest_costmap) to compute acc if desired
            acc_nom = np.array([0.0, float(ay), 0.0], dtype=float) 
            omega = np.array([0.0, 0.0, spin_yawrate], dtype=float)
            
            # TODO: check if the main net can be updated elsewhere and if CBF-QP can be run with higher frequency
            if latest_costmap is not None:
                orn_cbf.update_main_net(latest_costmap)
            acc = orn_cbf.cbf_qp(pos, vel, acc_nom)

            vel = vel + acc * dt
            pos = pos + vel * dt
            yaw += yawrate * dt

            cf.cmdFullState(
                pos + np.array(cf.initialPosition, dtype=float) + offset,
                vel,
                acc,
                yaw,
                omega
            )
            timeHelper.sleepForRate(rate)




def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # Subscribe to costmap on the underlying node and store latest
    def _on_costmap(msg: OccupancyGrid):
        setattr(timeHelper.node, 'latest_costmap', msg)
        print(f"Received costmap: resolution={msg.info.resolution}, "
              f"width={msg.info.width}, height={msg.info.height}")
    
    # Subscribe to the costmap topic (note the correct topic name with namespace)
    costmap_subscription = timeHelper.node.create_subscription(
        OccupancyGrid, 
        '/costmap/costmap',  # Updated topic name based on launch file
        _on_costmap, 
        1
    )
    
    # Give some time for the subscription to establish
    print("Waiting for costmap data...")
    timeHelper.sleep(2.0)

    rate = 30.0
    Z = 0.5
    yawrate = 2.0
    max_yaw_acc = 5.0
    # max_yaw_rate = 5.0

    # high-level mode test
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')
    cf.uploadTrajectory(0, 0, traj1)

    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # cf.goTo([0.0, 0.0, Z], 1.4, 2.0)
    # timeHelper.sleep(2.0)


    cf.setParam('hlCommander.yawacc', max_yaw_acc)
    cf.setParam('hlCommander.yawrlim', yawrate)
    cf.setParam('hlCommander.yawrate', yawrate)

    # executeTrajectory(timeHelper, cf,
    #                   Path(__file__).parent / 'data/figure8.csv',
    #                   rate,
    #                   offset=np.array([0, 0, 0.5]),
    #                   yawrate=yawrate)


    executeTrajectory2(timeHelper, cf, 
                    duration=4.0,
                    rate=rate,
                    offset=np.array([0, 0, 0.5]),
                    ay=-0.2,           
                    yawrate=yawrate)


    cf.startTrajectory(0)
    timeHelper.sleep(traj1.duration)

    # cf.goTo([0.0, 1.0, Z], 0.0, 5.0)
    # timeHelper.sleep(5.0)

    
    cf.setParam('hlCommander.yawrate', 0.0)
    timeHelper.sleep(2.0)

    # cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)


if __name__ == '__main__':
    main()
