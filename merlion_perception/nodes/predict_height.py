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

class PredictHeight(object):
    skip=3
    counter=0
    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 10)

        self.img_pub=rospy.Publisher('/gate_img', Image, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


    def img_callback(self, msg):

        if self.counter%self.skip==0:

            img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = img.shape[:2]
            # print(h,w)
            # img=self.img_correction(img)
            res=img.copy()
            blur = cv2.GaussianBlur(img,(7, 7),0)
            hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

            mask = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                        cv2.THRESH_BINARY,19, 2)
            
            kernel = np.ones((5,5),np.uint8)    
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            opening=255-opening
            opening = cv2.dilate(opening, None, iterations=1)

            #fit lines to extract major direction
            minLineLength=200
            lines = cv2.HoughLinesP(image=opening,rho=1,theta=np.pi/180,\
             threshold=100,lines=np.array([]), minLineLength=minLineLength, maxLineGap=12)
            
            grad=np.zeros((len(lines), 1))
            i=0
            for line in lines:
                #find two major gradients            
                x1, y1, x2, y2=line[0][0], line[0][1], line[0][2], line[0][3]
                theta=math.atan(float(y2-y1)/(x2-x1))*180/math.pi
                grad[i]=theta
                i+=1
                # cv2.line(res, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
            hist, bin_edges = np.histogram(grad, density=False)
            ind=np.argmax(hist)
            best_grad=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)
            # print(best_grad)

            #find area of rectangle
            contour_mask=255-opening

            M = cv2.getRotationMatrix2D((w/2,h/2),best_grad,1)
            contour_mask = cv2.warpAffine(contour_mask,M,(w,h))

            (_,contours,_) = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contour_mask=cv2.cvtColor(contour_mask, cv2.COLOR_GRAY2BGR)            
            areas=[]
            border=50
            for contour in contours:
                rect = cv2.boundingRect(contour)

                if rect[0]>border and rect[0]+rect[2]<w-border and rect[1]>border and rect[3]+rect[1]<h-border:
                    area=int(rect[3]*rect[2])
                    if area>1000:
                        cv2.rectangle(contour_mask, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), (0,255,0), 2)
                        areas.append(area)


            hist, bin_edges = np.histogram(np.asarray(areas), bins='fd', density=False)
            ind=np.argmax(hist)
            # best_area=(bin_edges[ind]+bin_edges[ind+1])/2
            best_area=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)
            pred_depth=self.predict_depth(best_area)

            print(best_grad, best_area, pred_depth)
            cv2.rectangle(contour_mask, (0, 0), (w, 80), (0,0,0), -1)


            font = cv2.FONT_HERSHEY_SIMPLEX
            color=(0, 0, 255)


            text="direction "+str(best_grad)+", height: "+str(pred_depth) +"m"
            cv2.putText(contour_mask, text, (50, 50), font, 0.8, color, 1, cv2.LINE_AA)

            opening=cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([res, contour_mask]), "bgr8"))

        self.counter+=1

    def predict_depth(self, area):
        if area<10:
            return 0
        #fov of camera
        fov_w, fov_h=60*math.pi/180, 45*math.pi/180
        px_W, px_H=640, 480

        #tile real world dimension
        real_w, real_h=0.2, 0.1

        #pixel tile size
        px_w=math.sqrt(area/(real_w*real_h))*real_w
        px_h=math.sqrt(area/(real_w*real_h))*real_h

        #camera fov in meters
        W=px_W*real_w/px_w
        H=px_H*real_h/px_h

        #predict depth
        d_w=W/(2*math.tan(fov_w/2))
        d_h=H/(2*math.tan(fov_h/2))

        print(d_w,d_h)

        return (d_w+d_h)/2


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
        PredictHeight(nodename="detect_gate", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
