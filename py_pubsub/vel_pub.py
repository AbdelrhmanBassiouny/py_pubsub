import rclpy
from rclpy.node import Node
import argparse
import sys


from geometry_msgs.msg import Twist


class VelPublisher(Node):

    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
    
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


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish robot velocities from command line.')
    parser.add_argument('--lin', metavar='l', type=float,
                        help='Linear Velocity',required=True, nargs='+')
    parser.add_argument('--ang', metavar='a', type=float,
                        help='Angular Velocity', required=True, nargs='+')
    parsed_args = parser.parse_args()
    lin_vel = parsed_args.lin
    ang_vel = parsed_args.ang
    print(lin_vel)
    print(ang_vel)

    if lin_vel is None:
        raise ValueError("Linear vel is None, It should be a float")
    if ang_vel is None:
        raise ValueError("Angular vel is None, It should be a float")
    
    rclpy.init(args=args)

    vel_publisher = VelPublisher()
    
    rclpy.spin_once(vel_publisher,timeout_sec=0.5)
    
    vel_publisher.pub_vel(lin_vel, ang_vel)

    try:
        rclpy.spin(vel_publisher)
    except KeyboardInterrupt:
        print('vel_publisher stopped cleanly')
        vel_publisher.pub_vel([0.0,0.0,0.0], [0.0,0.0,0.0])
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        vel_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()