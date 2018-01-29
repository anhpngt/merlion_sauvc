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

class DetectGate(object):


    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 10)

        self.img_pub=rospy.Publisher('/gate_img', Image, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


    def img_callback(self, msg):

        start_time=time.time()
        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # img=self.img_correction(img)
        print(img.shape)

        res=img.copy()
        blur = cv2.GaussianBlur(img,(7, 7),0)
        grey=cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)


        hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                    cv2.THRESH_BINARY,19, 2)
        kernel = np.ones((5,5),np.uint8)    
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        opening = cv2.dilate(opening, None, iterations=1)
        opening=255-opening

        # cv2.fastNlMeansDenoising(opening, opening, 7, 21, 3)

        # opening=self.dbscan(opening)

        minLineLength=100
        lines = cv2.HoughLinesP(image=opening,rho=1,theta=np.pi/180,\
         threshold=100,lines=np.array([]), minLineLength=minLineLength,maxLineGap=12)
        try:
            h_lines=[]
            v_lines=[]

            for line in lines:
                
                x1, y1, x2, y2=line[0][0], line[0][1], line[0][2], line[0][3]
                
                theta=abs(math.atan(float(y2-y1)/(x2-x1+0.001))*180/math.pi)
                # print(theta)
                angle_thres=30
                if theta<angle_thres:
                    #horizontal
                    cv2.line(res, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                    h_lines.append(np.array([[x1, y1], [x2, y2]]))
                elif abs(theta-90)<angle_thres:
                    #vertical
                    cv2.line(res, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)
                    v_lines.append(np.array([[x1, y1], [x2, y2]]))

            if len(h_lines)>0 and len(v_lines)>0:
                crosses=self.find_crosses(h_lines, v_lines)
                for cross, center in crosses:
                    # print(cross.shape)
                    cv2.circle(res, (int(cross[0]), int(cross[1])), 10, (255, 0, 0), -1)
                    cv2.circle(res, (int(center[0]), int(center[1])), 20, (170, 0, 170), -1)
        except:
            pass

        # dbscan=self.dbscan(opening)
        # self.img_pub.publish(self.bridge.cv2_to_imgmsg(dbscan, "mono8"))
        # return

        # #get vertical lines
        # edges_x = cv2.Sobel(grey,cv2.CV_8U,1,0,ksize=3) 
        # histogram_x = np.sum(edges_x, axis=0)

        # pointfour = np.int(0.4*histogram_x.shape[0])
        # pointsix=np.int(0.6*histogram_x.shape[0])
        # leftx_base = np.argmax(histogram_x[:pointfour]) #get the peak from first half(left)
        # rightx_base = np.argmax(histogram_x[pointsix:]) + pointsix #get peak from secondhalf(right)

        # font = cv2.FONT_HERSHEY_SIMPLEX
        # color=(0, 255, 100)

        # left_val=np.sum(histogram_x[leftx_base-20:leftx_base+20])
        # right_val=np.sum(histogram_x[rightx_base-20:rightx_base+20])

        # thres=50000

        # if left_val>thres:
        #     res=cv2.line(res, (leftx_base, 0), (leftx_base, img.shape[0]), (255, 0, 0), 2, 8, 0)
        #     cv2.putText(res, str(left_val), (leftx_base, 15), font, 0.4, color, 1, cv2.LINE_AA)
        #     edges_xf=edges_x.astype(np.float64)
        #     histogram_left = np.sum(edges_xf[:, leftx_base-20:leftx_base+20], axis=1)
        #     start, end=self.get_start_end(histogram_left)
        #     cv2.circle(res, (int(leftx_base), int(start)), 10, (255, 0, 0), -1)


        # if right_val>thres:
        #     res=cv2.line(res, (rightx_base, 0), (rightx_base, img.shape[0]), (255, 0, 0), 2, 8, 0)
        #     cv2.putText(res, str(right_val), (rightx_base, 15), font, 0.4, color, 1, cv2.LINE_AA)
        #     edges_xf=edges_x.astype(np.float64)
        #     histogram_right = np.sum(edges_x[:, rightx_base-20:rightx_base+20], axis=1)
        #     start, end=self.get_start_end(histogram_right)
        #     cv2.circle(res, (int(rightx_base), int(start)), 10, (255, 0, 0), -1)
        # #get horizontal lines

        # edges_y = cv2.Sobel(grey,cv2.CV_8U, 0, 1,ksize=3) 

        # histogram_y=np.sum(edges_y, axis=1)
        # y_base=np.argmax(histogram_y)
        # vertical_val=np.sum(histogram_y[y_base-15:y_base+15])

        # if vertical_val>thres:
        #     res=cv2.line(res, (0, y_base), (img.shape[1], y_base), (0, 0, 255), 2, 8, 0)
        #     cv2.putText(res, str(vertical_val), (15, y_base+10), font, 0.4, color, 1, cv2.LINE_AA)
        #     histogram_vert = np.sum(edges_y[y_base-20:y_base+20, :], axis=0)
        #     start, end=self.get_start_end(histogram_vert)
        #     cv2.circle(res, (int(start), int(y_base)), 10, (0, 0, 255), -1)
        # self.img_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        opening=cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([opening, res]), "bgr8"))

    def find_crosses(self, h_lines, v_lines):
        crosses=[]
        for h in h_lines:
            h2=h.copy()
            h2[[0, 1]]=h2[[1, 0]]
            h=np.concatenate((h, h2), axis=0)
            for v in v_lines:
                v=np.concatenate((v, v), axis=0)
                #find if they have near corners, four combinations
                d=np.sqrt(np.sum(np.square(h-v), axis=1))
                val=np.amin(d)
                if val<20:
                    ind=np.argmin(d)
                    cross=np.add(h[ind], v[ind])/2

                    center=np.add(h[(ind+2)%4], v[(ind+1)%4])/2

                    if center[1]>cross[1]:
                        crosses.append((cross, center))
        # print(crosses)
        return crosses


    def dbscan(self, mask):
    
        ind=np.where(mask!=0)
        # print(ind[0][np.newaxis].T)
        X=np.concatenate((ind[0][np.newaxis].T, ind[1][np.newaxis].T), axis=1)
        # print(X)

        #hierarchical dbscan
        clusterer = hdbscan.HDBSCAN(min_cluster_size=10)
        labels = clusterer.fit_predict(X)

        #normal dbscan
        # db = DBSCAN(eps=3, min_samples=20).fit(X)
        # core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        # core_samples_mask[db.core_sample_indices_] = True
        # labels = db.labels_
        # n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        
        unique_labels = set(labels)
        # print(unique_labels)

        res=np.zeros((mask.shape[0], mask.shape[1], 3))



        for k in unique_labels:
            if k==-1:
                continue

            ind=X[labels==k]
            # print(ind)
            if len(ind)<200:
                mask[ind[:, 0], ind[:, 1]]=0
            else:
                print(len(ind))
        return mask

    def get_start_end(self, hist):
        
        diff=hist[1:len(hist)]-hist[0:len(hist)-1]
        # diff=np.sort(diff)
        start=np.argmax(diff)
        # print(diff[0], diff[1])

        return start, 0

    def img_correction(self, img):
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        res=np.zeros_like(img)
        for i in range(3):
            res[:, :, i] = clahe.apply(img[:, :, i])
        return res
##########################
##########main############
##########################



if __name__ == '__main__':

    #load darknet
    # model.load_weights('/media/ml3/Volume/image-to-3d-bbox/weights.h5')

    try:
        DetectGate(nodename="detect_gate", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
