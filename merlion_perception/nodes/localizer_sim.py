#!/usr/bin/env python
#Reinaldo Maslim, NTU Merlion 2018

import rospy
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseArray, Vector3
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker
import tf
import time

import random

from tiles import Tile

#################################
##############class##############
#################################

class Localizer(object):
    x, y, z=0, 0, 0.8

    r, p, yaw=0, 0, 0


    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        ####Subscribers####
        #sub to downward cam as main for localizer
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size = 1)


        ####Publishers####
        self.vodom_pub=rospy.Publisher('/visual_odom', Odometry, queue_size=1)

        
        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pub_sim_odom()

            rate.sleep()

    def pub_sim_odom(self):

        #if it's the first time, memorize its initial readings
        br = tf.TransformBroadcaster()

        br.sendTransform((self.x, self.y, self.z),
                         tf.transformations.quaternion_from_euler(self.r, self.p, self.yaw),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        
        #publish odometry
        odom=Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x=self.x
        odom.pose.pose.position.y=self.y
        odom.pose.pose.position.z=self.z
        q=Quaternion()
        q.x, q.y, q.z, q.w=tf.transformations.quaternion_from_euler(self.r, self.p, self.yaw)
        odom.pose.pose.orientation=q
        self.vodom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        if msg.angular.x==50:
            return

        noise=random.random()*0.2-0.2/2
        self.x+=(msg.linear.x)*math.cos(self.yaw)-(msg.linear.y)*math.sin(self.yaw)#+noise
        noise=random.random()*0.2-0.2/2
        self.y+=msg.linear.x*math.sin(self.yaw)+msg.linear.y*math.cos(self.yaw)#+noise

        new_angle=self.yaw+msg.angular.z

        self.yaw=math.atan2(math.sin(new_angle), math.cos(new_angle))
        



##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        Localizer(nodename="localizer", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
