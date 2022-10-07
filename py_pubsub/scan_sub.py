import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import numpy as np


class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback, qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        self.get_logger().info("range_min is {}, range_max is {}".format(msg.range_min, msg.range_max))
        self.get_logger().info(
            "angle_increment is {}, angle_min is {}, angle_max is {}".format(msg.angle_increment, msg.angle_min, msg.angle_max))
        self.get_logger().info("time_inc is {}, time_bet_scans is {}".format(msg.time_increment, msg.scan_time))
        # msg.ranges = list(filter(lambda x: x>0, msg.ranges))
        print("argmin is {}".format(np.argmin(msg.ranges)))
        self.get_logger().info("min_max_range_found: {}, {}".format(min(msg.ranges), max(msg.ranges)))
        self.get_logger().info("min_max intensity: {}, {}".format(min(msg.intensities), max(msg.intensities)))
        self.get_logger().info("length of ranges is {}".format(len(msg.ranges)))


def main(args=None):
    rclpy.init(args=args)

    scan_subscriber = ScanSubscriber()

    rclpy.spin(scan_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()