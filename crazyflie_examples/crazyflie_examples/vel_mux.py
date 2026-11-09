#!/usr/bin/env python3
"""
Velocity multiplexer example using crazyflie_py.

Subscribes to /cmd_vel (Twist) and handles takeoff, hover, and landing
automatically, eliminating the need for the separate vel_mux ROS node.

Velocity mapping from the incoming Twist message:
  linear.x  -> vx (m/s forward)
  linear.y  -> vy (m/s left)
  angular.z -> yaw_rate (rad/s)
  linear.z  -> < 0 triggers landing; >= 0 continues hovering

Usage:
  ros2 run crazyflie_examples vel_mux --ros-args -p hover_height:=0.5
Then send commands with, e.g.:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from crazyflie_py import Crazyswarm
from geometry_msgs.msg import Twist
import rclpy


HOVER_HEIGHT = 0.5
TAKEOFF_DURATION = 2.0
LAND_HEIGHT = 0.05


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    has_taken_off = False
    received_first_cmd_vel = False
    msg_cmd_vel = Twist()

    def cmd_vel_callback(msg):
        nonlocal msg_cmd_vel, received_first_cmd_vel
        msg_cmd_vel = msg
        msg_is_zero = (
            msg.linear.x == 0.0
            and msg.linear.y == 0.0
            and msg.angular.z == 0.0
            and msg.linear.z == 0.0
        )
        if not msg_is_zero and not received_first_cmd_vel and msg.linear.z >= 0.0:
            received_first_cmd_vel = True

    swarm.allcfs.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)

    swarm.allcfs.get_logger().info(
        f'vel_mux ready for {cf.prefix}, hover height: {HOVER_HEIGHT} m'
    )

    while rclpy.ok():
        if received_first_cmd_vel and not has_taken_off:
            cf.takeoff(targetHeight=HOVER_HEIGHT, duration=TAKEOFF_DURATION)
            has_taken_off = True
            timeHelper.sleep(TAKEOFF_DURATION)

        if received_first_cmd_vel and has_taken_off:
            if msg_cmd_vel.linear.z >= 0:
                cf.cmdHover(
                    vx=msg_cmd_vel.linear.x,
                    vy=msg_cmd_vel.linear.y,
                    yaw_rate=msg_cmd_vel.angular.z,
                    z_distance=HOVER_HEIGHT,
                )
            else:
                cf.notifySetpointsStop()
                cf.land(targetHeight=LAND_HEIGHT, duration=TAKEOFF_DURATION)
                timeHelper.sleep(TAKEOFF_DURATION)
                has_taken_off = False
                received_first_cmd_vel = False

        timeHelper.sleepForRate(10)


if __name__ == '__main__':
    main()
