#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from std_srvs.srv import Empty
from crazyflie_interfaces.msg import Status
from std_msgs.msg import Bool

from scipy.spatial.transform import Rotation as R
import time
import numpy as np

class WatchDog(Node):

    def __init__(self):
        super().__init__('whatch_dog_node')
        self.declare_parameter('robot_prefix', '/C05')
        self.robot_prefix = self.get_parameter('robot_prefix').value
        poses_qos_deadline = 100.0  # example Hz

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            deadline=Duration(nanoseconds=int(1e9 / poses_qos_deadline))
        )
            #deadline = Duration(seconds=0, nanoseconds=1e9/100.0))

        self.create_subscription(
            NamedPoseArray, "/poses",
            self.watchdog_callback, qos_profile
        )
        self.create_subscription(Status, self.robot_prefix + '/status', self.status_callback, 10)

        self.publisher_landing = self.create_publisher(Bool, 'landing', 10)
 # prevent unused variable warning\
        self.emergency_client = self.create_client(Empty, self.robot_prefix + '/emergency')
        self.threshold = 40
        self.reboot_client = self.create_client(Empty, self.robot_prefix + '/reboot')
        time.sleep(5.0)

    def watchdog_callback(self, msg):
        poses = msg.poses
        for pose in poses:
            if pose.name == self.robot_prefix:
                quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                r = R.from_quat(quat).as_euler('zyx', degrees=True)
                if np.abs(r[1]) > self.threshold or np.abs(r[2]) > self.threshold:
                    self.get_logger().info('Roll or Pitch is too high')
                    land = Bool()
                    land.data = True
                    self.publisher_landing.publish(land)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    
    def reboot(self):
        req = Empty.Request()
        self.reboot_client.call_async(req)
        time.sleep(1.0)    

    def status_callback(self, msg):
        if msg.battery_voltage < 2.8:
            self.get_logger().info('Battery voltage is too low')
            land = Bool()
            land.data = True
            self.publisher_landing.publish(land)

def main(args=None):
    rclpy.init(args=args)

    watchdog = WatchDog()

    rclpy.spin(watchdog)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchdog.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()