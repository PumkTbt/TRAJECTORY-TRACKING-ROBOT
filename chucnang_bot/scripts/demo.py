#! /usr/bin/env python3
#!/bin/sh
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
class Driver_node(Node):

    def __init__(self):
        super().__init__('driving_custom_Node')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(1, self.timer_callback)



    def timer_callback(self):
        msg = Twist()
        # W = v / r
        l = -0.5
        a = 1
        linear_vel= float(l)
        radius = float(a)
        msg.linear.x=linear_vel
        msg.linear.y= 0.0
        msg.angular.z=linear_vel/radius
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Driver_node()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()