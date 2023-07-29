import time 
from geometry_msgs.msg import PoseStamped 
from rclpy.duration import Duration 
import rclpy
from robot_navigator import BasicNavigator, NavigationResult 

def main():
  rclpy.init()
  navigator = BasicNavigator()
  navigator.waitUntilNav2Active()
  #list_pose_x = [1.159]
  #list_pose_y = [-0.254]
  #list_pose_z = [0.09]

  #list_orien_x = [0.537878]
  #list_orien_y = [-0.1303084]
  #list_orien_z = [-0.0319516]
  #list_orien_w = [0.8322776]
  
  goal_poses = []
   
  for i in range(0,len(list_pose_x)):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.159
    goal_pose.pose.position.y = -0.254
    goal_pose.pose.position.z = 0.09
    goal_pose.pose.orientation.x = 0.537878
    goal_pose.pose.orientation.y = -0.1303084
    goal_pose.pose.orientation.z = -0.0319516
    goal_pose.pose.orientation.w = 0.8322776
    goal_poses.append(goal_pose)
  
  navigator.goThroughPoses(goal_poses)
 
  i = 0
  while not navigator.isNavComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')

      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1000000.0):
        navigator.cancelNav()
 
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500000.0):
        goal_pose_alt = PoseStamped()
        goal_pose_alt.header.frame_id = 'map'
        goal_pose_alt.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_alt.pose.position.x = -6.5
        goal_pose_alt.pose.position.y = -4.2
        goal_pose_alt.pose.position.z = 0.0
        goal_pose_alt.pose.orientation.x = 0.0
        goal_pose_alt.pose.orientation.y = 0.0  
        goal_pose_alt.pose.orientation.z = 0.0
        goal_pose_alt.pose.orientation.w = 1.0
        navigator.goThroughPoses([goal_pose_alt])

  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
  elif result == TaskResult.FAILED:
    print('Goal failed!')
  else:
    print('Goal has an invalid return status!')
 
  navigator.lifecycleShutdown()
  exit(0)
 
if __name__ == '__main__':
  main()
