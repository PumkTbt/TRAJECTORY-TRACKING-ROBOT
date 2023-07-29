import math 
import rclpy 
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist       
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 
 
import numpy as np 
 
class Controller(Node):
  def __init__(self):
    super().__init__('Controller')
    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/demo/state_est',
                        self.state_estimate_callback,
                        10)
    self.subscription  # prevent unused variable warning
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/scan',
                           self.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)
 
    self.max_distance = 3.5
    self.crashed_min = 0.30
    self.crashed = False

    self.left_dist = self.max_distance # Left
    self.leftfront_dist = self.max_distance # Left-front

    self.front_dist = self.max_distance
    self.front_dist_1 = self.max_distance # Front
    self.front_dist_2 = self.max_distance # Front

    self.behind_dist = self.max_distance # Behind

    self.rightfront_dist = self.max_distance # Right-front
    self.right_dist = self.max_distance # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.25
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0
 
    ############# WALL FOLLOWING PARAMETERS #######################     
    # Finite states for the wall following mode
    #  "turn left": Robot turns towards the left
    #  "search for wall": Robot tries to locate the wall        
    #  "follow wall": Robot moves parallel to the wall
    #self.wall_following_state = "turn left"
         
    # Set turning speeds (to the left) in rad/s 
    # These values were determined by trial and error.
    self.turning_speed_wf_fast = 0.5  # Fast turn
    #self.turning_speed_wf_slow = 0.2 # Slow turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 0.45 # in meters  
 
    # We don't want to get too close to the wall though.
    self.dist_too_close_to_wall = 0.20 # in meters

  # def logger_callback(self):
  #   self.get_logger().info('Publishing: "%lf"' % front_distance)
 
  def state_estimate_callback(self, msg):
    """
    Extract the position and orientation data. 
    This callback is called each time
    a new message is received on the '/demo/state_est' topic
    """
    # Update the current estimated state in the global reference frame
    curr_state = msg.data
    self.current_x = curr_state[0]
    self.current_y = curr_state[1]
    self.current_yaw = curr_state[2]
 
    # Command the robot to keep following the wall      
    self.follow_wall()
    
 
    # Read the laser scan data that indicates distances
    # to obstacles (e.g. wall) in meters and extract
    # 5 distinct laser readings to work with.
    # Each reading is separated by 45 degrees.
    # Assumes 181 laser readings, separated by 1 degree. 
    # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
 
    #number_of_laser_beams = str(len(msg.ranges))       
    #self.left_dist = msg.ranges[90]
    #self.leftfront_dist = msg.ranges[45]
    #self.front_dist = msg.ranges[0]
    #self.rightfront_dist = msg.ranges[315]
    #self.right_dist = msg.ranges[270]
 
  def scan_callback(self, msg):

    
    self.left_dist = min(msg.ranges[60:100])
    self.leftfront_dist = min(msg.ranges[30:60])
    
    self.rightfront_dist = min(msg.ranges[300:330])
    self.right_dist = min(msg.ranges[260:300])

    self.front_dist_1 = min(msg.ranges[0:30])
    self.front_dist_2 = min(msg.ranges[330:359])
    self.front_dist = min(self.front_dist_1, self.front_dist_2)

    self.behind_dist = min(msg.ranges[180:270])

    if self.front_dist_1 < self.crashed_min or self.rightfront_dist < self.crashed_min or self.right_dist < self.crashed_min :
      if self.behind_dist > 1.0 :
        self.crashed = True
      else :
        self.crashed = False  
   
    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam 
   
  def follow_wall(self):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    d = self.dist_thresh_wf
    e = self.dist_too_close_to_wall

    if self.front_dist > e and self.rightfront_dist > e and self.right_dist > e:
      if not self.crashed: 
        if self.front_dist > d and self.leftfront_dist > d and self.rightfront_dist > d:
          self.wall_following_state = "search for wall"
          msg.linear.x = self.forward_speed
          msg.angular.z = 0.0
    
        elif self.front_dist < d and self.leftfront_dist > d  and self.rightfront_dist > d:
          self.wall_following_state = "turn left"
          msg.linear.x = 0.0
          msg.angular.z = self.turning_speed_wf_fast
    
        elif self.front_dist > d and self.leftfront_dist > d and self.rightfront_dist < d:
          self.wall_following_state = "turn left"
          msg.linear.x = 0.0
          msg.angular.z = self.turning_speed_wf_fast      
    
        elif self.front_dist > d and self.leftfront_dist < d and self.rightfront_dist > d:
          self.wall_following_state = "turn right"
          msg.linear.x = 0.0
          msg.angular.z = -self.turning_speed_wf_fast 
    
        elif self.front_dist < d and self.leftfront_dist > d and self.rightfront_dist < d:
          self.wall_following_state = "turn left"
          msg.linear.x = 0.0
          msg.angular.z = self.turning_speed_wf_fast
    
        elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist > d:
          self.wall_following_state = "turn right"
          msg.linear.x = 0.0
          msg.angular.z = -self.turning_speed_wf_fast
    
        elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist < d:
          self.wall_following_state = "turn left"
          msg.linear.x = 0.0
          msg.angular.z = self.turning_speed_wf_fast
                
        elif self.front_dist > d and self.leftfront_dist < d and self.rightfront_dist < d:
          self.wall_following_state = "turn left"
          msg.linear.x = 0.0
          msg.angular.z = self.turning_speed_wf_fast 
      
      else :
        self.wall_following_state = "move backward"
        msg.linear.x = -1.0
        msg.angular.z = 0.0

    else:
      msg.angular.x = 0.0
      msg.angular.z = 0.0
    self.publisher_.publish(msg)  
    self.get_logger().info('Robot State: "%s"' % self.wall_following_state)
    
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
