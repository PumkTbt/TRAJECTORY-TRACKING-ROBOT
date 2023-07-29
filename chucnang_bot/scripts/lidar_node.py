import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time 

# pr_tg = 0

class ObstacleAvoidance(Node):

    pr_tg = 0

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.twist = Twist()
        self.initial_velocity = 0.2  # initial velocity
        self.min_velocity = 0.2  # minimum velocity

    def scan_callback(self, scan):
        # Define a threshold for obstacles
        threshold = 1.0
        allDist=[]
        for i in range(15):
            if scan.ranges[i] < 0.2:
                allDist.append(scan.ranges[int(i)])
        
        if len(allDist)!=0:
            minDist = min(allDist)
            # print("t:",min(allDist))
            now = time.localtime()
            tg = now.tm_min * 60 + now.tm_sec
            q = tg - self.pr_tg
            self.pr_tg = tg
            print("minDist: ",minDist,"q:",q," tg: ",tg)
        # Get the minimum distance from the laser scan data
        min_distance = min(scan.ranges)

        # print(min_distance)

        # Calculate velocity based on the distance from obstacles
        if min_distance < threshold:
            velocity = self.min_velocity
        else:
            velocity = self.initial_velocity

        # Turn left if an obstacle is detected
        if min_distance < threshold:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            # Move forward with the calculated velocity
            self.twist.linear.x = velocity
            self.twist.angular.z = 0.0

        # Publish the twist command
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()