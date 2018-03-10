#!/usr/bin/python3
import math
import numpy as np

# class Mission1(object):
#   def __init__(self, nodename):
#     # Initialize node
#     rospy.init_node(nodename, anonymous=False)
#     rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self.callBack, queue_size=1)
#     cmdvel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

#     # Mission 1 state
#     self.state = 0 # state 0 for depth, state 1 forward, state 2 backward, state 4 finished
    
#     # Run
#     rospy.spin()

#   def callBack(self):

def angle_diff(minuend, subtrahend):
  diff = minuend - subtrahend
  if diff > math.pi:
    diff -= 2 * math.pi
  elif diff < -math.pi:
    diff += 2 * math.pi
  return diff

if __name__ == '__main__':
  # try:
  #   node = Mission1(nodename='Mission1')
  # except rospy.ROSInterruptException:
  #   pass

  # print(angle_diff(-3.1, 3.0))
  pos = [[1], [2]]
  yaw = 1.0
  matrix = [[math.cos(yaw), math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
  result = np.matmul(matrix, pos)
  print(result)