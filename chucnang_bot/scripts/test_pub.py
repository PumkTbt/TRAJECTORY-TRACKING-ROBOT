#! /usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pub_path')
        self.publisher_ = self.create_publisher(Path, 'nav2/path_pub', 1)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        message = Path()
        message.header = message.header
        pose = PoseStamped()
        pose.header = self.header
        pose.pose = self.pose.pose
        message.poses.append(pose)
        self.publisher_.publish(message)
        self.i += float(sys.argv[3])

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()