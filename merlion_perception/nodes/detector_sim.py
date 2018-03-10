#!/usr/bin/env python
#NTU Merlion 2018

import rospy
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseArray
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker

import time
import random


#################################
##############class##############
#################################

class Detector(object):
    #odom
    x0, y0, z0=0, 0, 0
    roll0, pitch0, yaw0=0, 0, 0
    odom_received=False

    #front cam c920 parameters
    fov_w, fov_h=62*math.pi/180, 46*math.pi/180
    px_W, px_H=640, 480

    #make birdeye heatmap with size 50, 25, ppm=2, init_pos=0.7, 25 
    #3 channels for gate, blue bucket, flare
    heatmaps=np.zeros((50, 100, 3), dtype=np.uint8)
    init_pos=0.7, 25
    ppm=2

    #trickiest param for gate, cloudy water use 1, if clear use 2
    erode=2

    #simulated global positions of detection items
    detections =[[5, 1], [23, 6], [15, -5]]

    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()
        self.init_markers()
        ####Subscribers####

        #sub odom
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)
        while not self.odom_received and not rospy.is_shutdown():
            rospy.sleep(1)
            print("waiting for odom...")

        ####Publishers####
        #pub images for debugging
        self.birdeye_pub=rospy.Publisher('/detection/birdeye', Image, queue_size=1)

        #pub heatmap of detections with 3 channels for mission planner
        self.heatmap_pub=rospy.Publisher('/detection/heatmap', Image, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            #publish 

            #clean up noise of heatmaps
            # self.heatmaps[self.heatmaps<2]=0
            self.pub_sim_heatmap()

            #pub birdeye of combined heatmap, only for visualization
            max_val=50
            birdeye=np.clip(np.sum(self.heatmaps, axis=2).astype(np.uint8), 0, max_val)                
            if max_val>0:
                birdeye = cv2.applyColorMap(birdeye*int(255/max_val), cv2.COLORMAP_JET)
            else:
                birdeye = cv2.applyColorMap(birdeye, cv2.COLORMAP_JET)  

            ind_x=int(self.heatmaps.shape[0]-(self.init_pos[0]+self.x0)*self.ppm)
            ind_y=int((self.init_pos[1]-self.y0)*self.ppm)
            birdeye[ind_x, ind_y]=[255, 255, 255]
            self.birdeye_pub.publish(self.bridge.cv2_to_imgmsg(birdeye, "bgr8"))

            #pub 3 channel heatmap for mission planner
            self.heatmaps=np.clip(self.heatmaps, 0, 255)
            self.heatmap_pub.publish(self.bridge.cv2_to_imgmsg(self.heatmaps, "bgr8"))

            rate.sleep()

    def pub_sim_heatmap(self):
        r=8
        blindspot_r=2
        self.printMarker(self.detections)
        for i in range(len(self.detections)):
            detection=self.detections[i]
            theta=math.atan2(detection[1]-self.y0, detection[0]-self.x0)-self.yaw0
            theta=math.atan2(math.sin(theta), math.cos(theta))
            dist=math.sqrt((self.x0-detection[0])**2+(self.y0-detection[1])**2)
            std_dev=float(dist)/8

            if dist<r and abs(theta)<self.fov_w/2 and dist>blindspot_r:
                noise=(random.random()-0.5)*std_dev
                x=detection[0]+noise
                noise=(random.random()-0.5)*std_dev
                y=detection[1]+noise

                ind_x=int(self.heatmaps.shape[0]-(self.init_pos[0]+x)*self.ppm)
                ind_y=int((self.init_pos[1]-y)*self.ppm)
                self.heatmaps[ind_x, ind_y, i]+=1
                


    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.z0 = msg.pose.pose.position.z
        # print(self.z0)
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        


    def printMarker(self, pos_list):
                #markerList store points wrt 2D world coordinate
                
        self.markers.points=[]
        for pos in pos_list:
            p=Point()

            self.markers.points.append(Point(0, 0, 0))
            p.x=pos[0]
            p.y=pos[1]
            p.z=0.75
            self.markers.points.append(p)

        self.marker_pub.publish(self.markers)


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'frontiers'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('/detection', Marker, queue_size=5)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        # self.markers.type = Marker.ARROW
        self.markers.type = Marker.SPHERE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.scale.z = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

        p=Point()
        p.x=0
        p.y=0
        p.z=0
        q_angle = quaternion_from_euler(0, 0, 0)
        q = Quaternion(*q_angle)
        self.markers.pose = Pose(p, q)

##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        Detector(nodename="detector", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
