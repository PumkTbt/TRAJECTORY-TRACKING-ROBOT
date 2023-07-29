import numpy as np 

def get_quaternion_from_euler(roll, pitch, yaw):
  """
    + roll (goc xoay quanh truc x)
    + pitch (goc xoay quanh truc y) 
    + yaw (goc xoay quanh truc z)
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  """
     + qx, qy, qz, qw: Dinh dang vi tri 3D sang quaternion [x,y,z,w]
  """
  return [qx, qy, qz, qw]
