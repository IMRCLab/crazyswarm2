import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.msg import Obstacle, ObstacleArray


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher')

        all_cfs = ["cf5", "cf231"]
        self.msg_out = ObstacleArray()
        
        self.subscriptions = []
        for cf in all_cfs:
            self.msg_out.obstacles.append(Obstacle())
            self.msg_out.obstacles[-1].id = cf
            self.subscriptions.append(self.create_subscription(
                LogDataGeneric,
                'cf5/distate',
                lambda msg: self.listener_callback(msg, len(self.msg_out.obstacles)-1),
                10))
            
        self.publisher_ = self.create_publisher(ObstacleArray, 'obstacles', 10)
        self.timer = self.create_timer(1/10.0, self.timer_callback)


    def listener_callback(self, msg, idx):
        self.msg_out[idx].pose.position.x = msg.data.values[0]
        self.msg_out[idx].pose.position.y = msg.data.values[1]
        self.msg_out[idx].linear.position.x = msg.data.values[2]
        self.msg_out[idx].linear.position.y = msg.data.values[3]
        self.msg_out[idx].accel.x = msg.data.values[4]
        self.msg_out[idx].accel.y = msg.data.values[5]

    def timer_callback(self):
        self.publisher_.publish(self.msg_out)


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = ObstaclePublisher()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()