#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math



class TfBroadcaster(object):
    initialize_localizer=True

    #the cg of robot when firstly launched wrt to map frame
    cg_origin=[0, 0]


    def __init__(self, nodename):
        rospy.init_node('tf_broadcaster')
        listener = tf.TransformListener()

        r = rospy.Rate(100)

        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()

            #will be given by rplidar slam gmapping
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "odom",
                             "map")

            r.sleep()








if __name__ == '__main__':
    try:
        TfBroadcaster(nodename="tf")
    except rospy.ROSInterruptException:
        rospy.loginfo("tf broadcaster exit.")

