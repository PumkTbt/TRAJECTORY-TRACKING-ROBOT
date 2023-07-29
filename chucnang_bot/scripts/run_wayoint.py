#! /usr/bin/env python3
import time 
from copy import deepcopy

from geometry_msgs.msg import PoseStamped 
from rclpy.duration import Duration 
import rclpy 
from robot_navigator import BasicNavigator, NavigationResult 
# Chuyen doi thong so 3D dang Euler (x,y,z) sang quaternion (x,y,z,w)
from chucnang_bot.euler_to_quaternion import get_quaternion_from_euler 

def main():
  rclpy.init()
  navigator = BasicNavigator()
  inspection_route = [
    [-1.855, -1.742, 0.0935],
    [1.55093, -4.847, 0.0074],
    [3.780, 3.8143,0.0046],
    [-4.722, 0.6253,0.00413]]

  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'map'
  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  initial_pose.pose.position.x = -1.855
  initial_pose.pose.position.y = -1.742
  initial_pose.pose.position.z = 0.0

  q = get_quaternion_from_euler(0.0, 0.0,0.0935)     
  initial_pose.pose.orientation.x = q[0]
  initial_pose.pose.orientation.y = q[1]
  initial_pose.pose.orientation.z = q[2]
  initial_pose.pose.orientation.w = q[3]
  # navigator.setInitialPose(initial_pose)

  navigator.waitUntilNav2Active()

  inspection_points = []
  inspection_pose = PoseStamped()
  inspection_pose.header.frame_id = 'map'
  inspection_pose.header.stamp = navigator.get_clock().now().to_msg()

  for pt in inspection_route:
 
    inspection_pose.pose.position.x = pt[0]
    inspection_pose.pose.position.y = pt[1]
    inspection_pose.pose.position.z = 0.0
 
    quat = get_quaternion_from_euler(0.0, 0.0, pt[2])     
    inspection_pose.pose.orientation.x = quat[0]
    inspection_pose.pose.orientation.y = quat[1]
    inspection_pose.pose.orientation.z = quat[2]
    inspection_pose.pose.orientation.w = quat[3]    
    inspection_points.append(deepcopy(inspection_pose))
  
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(inspection_points)

  i = 0
  while not navigator.isNavComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Da thuc hien: ' +
        str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points))+" muc tieu")

  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Hoan thanh muc tieu.')
  elif result == NavigationResult.CANCELED:
    print('Canceled')
    exit(1)
  elif result == NavigationResult.FAILED:
    print('Failed')

  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  navigator.goToPose(initial_pose)
  while not navigator.isNavComplete():
    pass

  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Da quay ve diem xuat phat.')
  elif result == NavigationResult.CANCELED:
    print('Cancle')
    exit(1)
  elif result == NavigationResult.FAILED:
    print('Failed')
  
  
  exit(0)

if __name__ == '__main__':
    main()