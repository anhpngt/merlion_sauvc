import rospy
import roslib
from std_msgs.msg import Bool

# import serial
import sys
# import string
import struct
import time
import math
import roslaunch
import subprocess
import signal


mission_state = False
last_disarm_cmd = []
save_size = 20

def disarm_cb(msg):
	global mission_state
	global last_disarm_cmd
	global mission_process

	last_disarm_cmd.append(msg.data)
	if len(last_disarm_cmd) >= save_size:
		last_disarm_cmd.pop(0)

	# if disarm -> arm	
	if all(item == True for item in last_disarm_cmd[:10]) and all(item == False for item in last_disarm_cmd[10:]) and mission_state == False:
		time.sleep(5)
		mission_process = subprocess.Popen(["roslaunch", "merlion_bringup", "mission_planner.launch"])
		mission_state = True
	elif all(item == False for item in last_disarm_cmd[:10]) and all(item == True for item in last_disarm_cmd[10:]) and mission_state == True:
		mission_process.terminate()
		mission_process.wait()
		mission_state = False

if __name__ == '__main__':
	rospy.init_node('mission_overwatch', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

	roslaunch.configure_logging(uuid)

	disarm_sub = rospy.Subscriber('/merlion/disarm', Bool, disarm_cb)
	while not rospy.is_shutdown():
		rospy.spin()