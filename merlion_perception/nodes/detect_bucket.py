#!/usr/bin/env python

import rospy
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseArray
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry

from visualization_msgs.msg import MarkerArray, Marker

import time

from sklearn.cluster import DBSCAN, KMeans
from sklearn import metrics
from sklearn.metrics.pairwise import euclidean_distances, manhattan_distances
from sklearn.preprocessing import StandardScaler, normalize
import random

import hdbscan

#################################
##############class##############
#################################

class DetectBucket(object):
    skip=3
    counter=0
    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 10)

        self.front_img_pub=rospy.Publisher('/bucket_img_front', Image, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


    def img_callback(self, msg):

        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        cimg=self.detect_circles(gray)


        self.front_img_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "bgr8"))

    def draw_circles(self, img, circles):
        # img = cv2.imread(img,0)
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        for i in circles[0,:]:
        # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            cv2.putText(cimg,str(i[0])+str(',')+str(i[1]), (i[0],i[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 255)
        return cimg

    def detect_circles(self, gray):
        gray_blur = cv2.medianBlur(gray, 13)  # Remove noise before laplacian
        gray_lap = cv2.Laplacian(gray_blur, cv2.CV_8UC1, ksize=5)
        dilate_lap = cv2.dilate(gray_lap, (3, 3))  # Fill in gaps from blurring. This helps to detect circles with broken edges.
        # Furture remove noise introduced by laplacian. This removes false pos in space between the two groups of circles.
        lap_blur = cv2.bilateralFilter(dilate_lap, 5, 9, 9)
        # Fix the resolution to 16. This helps it find more circles. Also, set distance between circles to 55 by measuring dist in image.
        # Minimum radius and max radius are also set by examining the image.
        circles = cv2.HoughCircles(lap_blur, cv2.cv.CV_HOUGH_GRADIENT, 16, 55, param2=450, minRadius=20, maxRadius=40)
        cimg = self.draw_circles(gray, circles)
        print("{} circles detected.".format(circles[0].shape[0]))
        # There are some false positives left in the regions containing the numbers.
        # They can be filtered out based on their y-coordinates if your images are aligned to a canonical axis.
        # I'll leave that to you.
        return cimg


    def img_correction(self, img):
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        res=np.zeros_like(img)
        for i in range(3):
            res[:, :, i] = clahe.apply(img[:, :, i])
        return res

    def rand_color(self):
        a = random.randrange(0,256)
        b = random.randrange(0,256)
        c = random.randrange(0,256)

        return np.array([a, b, c])

##########################
##########main############
##########################



if __name__ == '__main__':

    #load darknet
    # model.load_weights('/media/ml3/Volume/image-to-3d-bbox/weights.h5')

    try:
        DetectBucket(nodename="detect_bucket", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
