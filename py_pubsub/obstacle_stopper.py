import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import argparse
import sys
from time import sleep


class ObstacleStopper(Node):
    # This class is for the Wandering Behaviour of the turtlebot (It can easily be changed to stop instead of wander)
    def __init__(self):
        super().__init__('obstacle_stopper')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback, qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
    
    def pub_vel(self, lin_vel, ang_vel):
        msg = Twist()
        msg.linear.x = lin_vel[0]
        msg.linear.y = lin_vel[1]
        msg.linear.z = lin_vel[2]
        msg.angular.x = ang_vel[0]
        msg.angular.y = ang_vel[1]
        msg.angular.z = ang_vel[2]
        self.publisher.publish(msg)
        self.get_logger().info("Publishing lin_vel:{} ang_vel:{}".format(msg.linear, msg.angular))

    def scan_callback(self, msg):
        # Take n degrees from front as query area (n/2 from left to n/2 from right)
        n = 90
        a = int(n / 2)
        forward_ranges_left = msg.ranges[:a]
        forward_ranges_right = msg.ranges[359-a:]
        forward_ranges_right.extend(forward_ranges_left)
        forward_ranges = forward_ranges_right
        
        # filter data to remove outliers (outside range of sensor)
        forward_ranges = list(filter(lambda x: msg.range_max >=
                          x >= msg.range_min, forward_ranges))
        min_range = min(forward_ranges)
        
        self.get_logger().info("min_max_range_found: {}, {}".format(min_range, max(msg.ranges)))
        # self.get_logger().info("min_max intensity: {}, {}".format(min(msg.intensities), max(msg.intensities)))
        
        # Query min range infront of the robot to detect and avoid obstacles
        if min_range <= 0.3:
            self.pub_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.2])
        else:
            self.pub_vel([0.1, 0.0, 0.0], [0.0, 0.0, 0.0])


def main(args=None):
    
    rclpy.init(args=args)

    obstacle_stopper = ObstacleStopper()

    # Here for safety reasons if code is stopped manually I send a zero velocity to stop robot
    try:
        rclpy.spin(obstacle_stopper)
    except KeyboardInterrupt:
        print('vel_publisher stopped cleanly')\
        # sometimes it doesnt stop from one command so I send two with some delay between them
        obstacle_stopper.pub_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        sleep(0.5)
        obstacle_stopper.pub_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        obstacle_stopper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()