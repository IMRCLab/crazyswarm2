import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.msg import Obstacle, ObstacleArray


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__("obstacle_publisher")

        all_cfs = ["cf1", "cf5", "cf6", "cf7", "cf8"]
        self.msg_out = ObstacleArray()

        # self.subscriptions_ = []
        name_to_k = dict()
        for k, cf in enumerate(all_cfs):
            self.msg_out.obstacles.append(Obstacle())
            self.msg_out.obstacles[-1].id = cf
            name_to_k[cf] = k

        self.subs = map(
            lambda cf: self.create_subscription(
                LogDataGeneric,
                f"{cf}/distate",
                lambda msg: self.listener_callback(msg, name_to_k[cf], cf),
                10,
            ),
            all_cfs,
        )
        self.subs = list(self.subs)

        # for cf in all_cfs:
        #     self.msg_out.obstacles.append(Obstacle())
        #     self.msg_out.obstacles[-1].id = cf
        #     sub_ = self.create_subscription(
        #         LogDataGeneric,
        #         cf + '/distate',
        #         lambda msg: self.listener_callback(msg, len(self.msg_out.obstacles)-1, cf),
        #         10)
        #     self.subscriptions_.append(sub_)

        # self.sub1_ = self.create_subscription(
        #         LogDataGeneric,
        #         f'cf1/distate',
        #         lambda msg: self.listener_callback(msg, 0, "cf1"),
        #         10)

        # self.sub2_ = self.create_subscription(
        #         LogDataGeneric,
        #         f'cf5/distate',
        #         lambda msg: self.listener_callback(msg, 1, "cf5"),
        #         10)

        self.publisher_ = self.create_publisher(ObstacleArray, "obstacles", 10)
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    def listener_callback(self, msg, idx, cf):
        print(idx, cf)
        self.msg_out.obstacles[idx].pose.position.x = msg.values[0]
        self.msg_out.obstacles[idx].pose.position.y = msg.values[1]
        self.msg_out.obstacles[idx].twist.linear.x = msg.values[2]
        self.msg_out.obstacles[idx].twist.linear.y = msg.values[3]
        self.msg_out.obstacles[idx].accel.x = 0.0  # msg.values[4]
        self.msg_out.obstacles[idx].accel.y = 0.0  # msg.values[5]

    def timer_callback(self):
        self.publisher_.publish(self.msg_out)


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = ObstaclePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
