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
    forward_speed=0.5
    side_speed=0.5
    dive_speed=0.2
    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 10)
        self.img_pub=rospy.Publisher('/gate_img', Image, queue_size=1)

        self.cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


    def img_callback(self, msg):
        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)
        start_time=time.time()
        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # img=self.img_correction(img)
        
        res=img.copy()
        blur = cv2.GaussianBlur(img,(7, 7),0)
        grey=cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)


        hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                    cv2.THRESH_BINARY,21, 2)
        kernel = np.ones((5,5),np.uint8)    
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        opening=255-opening
        opening = cv2.erode(opening, None, iterations=1)
        

        # cv2.fastNlMeansDenoising(opening, opening, 7, 21, 3)

        # opening=self.dbscan(opening)

        minLineLength=70
        lines = cv2.HoughLinesP(image=opening,rho=1,theta=np.pi/180,\
         threshold=80,lines=np.array([]), minLineLength=minLineLength,maxLineGap=12)

        heatmap=np.zeros_like(opening)
        side=10
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
                
                for cross, center, depth in crosses:
                    cv2.circle(res, (int(cross[0]), int(cross[1])), 10, (255, 0, 0), -1)
                    cv2.circle(res, (int(center[0]), int(center[1])), 20, (170, 0, 170), -1)
                    heatmap[center[1]-side:center[1]+side, center[0]-side:center[0]+side]+=1
                    text="distance: "+str(round(depth, 2)) +"m"
                    cv2.putText(res, text, (int(cross[0])+10, int(cross[1])-20), font, 0.5, color, 1, cv2.LINE_AA)

            #process heatmap
            gate=(np.argmax(heatmap)%img.shape[1], int(math.floor(np.argmax(heatmap)/img.shape[1])+1))
            max_val=np.amax(heatmap)
            if max_val>0:
                heatmap_img = cv2.applyColorMap(heatmap*int(255/max_val), cv2.COLORMAP_JET)
            else:
                heatmap_img = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
            #publish velocity command  
            msg=Twist()
            print(opening.shape)
            print(gate)

            pt1=(int(opening.shape[1]/2), int(opening.shape[0]/2))
            if max_val==0:
                #stop
                msg.linear.x=0
                msg.linear.y=0
            elif abs(gate[1]-opening.shape[0]/2.0)/opening.shape[0]>0.2:
                #adjust height 
                sign=abs(gate[1]-opening.shape[0]/2.0)/(gate[1]-opening.shape[0]/2.0)
                #move sideway
                msg.linear.z=-1.0*sign*self.dive_speed
                pt2=(pt1[0], pt1[1]+int(sign*100))
                cv2.arrowedLine(res, pt1, pt2, (0,230,235), 5)
            elif abs(gate[0]-opening.shape[1]/2.0)/opening.shape[1]<0.1:
                #move forward
                msg.linear.x=self.forward_speed
                pt2=(pt1[0], pt1[1]-30)
                cv2.arrowedLine(res, pt1, pt2, (0,0,255), 5)
            else:
                sign=abs(gate[0]-opening.shape[1]/2.0)/(gate[0]-opening.shape[1]/2.0)
                #move sideway
                msg.linear.y=-1.0*sign*self.side_speed
                pt2=(pt1[0]+int(sign*100), pt1[1])
                cv2.arrowedLine(res, pt1, pt2, (255,0,0), 5)
            

            
            self.cmd_vel_pub.publish(msg)

        except:
            pass

        opening=cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
        fin = cv2.addWeighted(heatmap_img, 1, opening, 1, 0)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([fin, res]), "bgr8"))

    def predict_depth(self, line1, line2):
        fov_w, fov_h=60*math.pi/180, 45*math.pi/180
        px_W, px_H=640, 480

        # print(line1, line2)
        # print(np.subtract(line1[0, :], line1[1,:]))
        l1=np.sqrt(np.sum(np.square(np.subtract(line1[0, :], line1[1,:])), axis=0))
        l2=np.sqrt(np.sum(np.square(np.subtract(line2[0, :], line2[1,:])), axis=0))
        # print(l1, l2)
        if abs(l2-l1)/l2<0.3:
            l=(l1+l2)/2
        elif abs(l2-l1)/l2<0.5:
            l=max(l1, l2)
        else:
            #not pole
            return -1
        #real length of pole in metres
        real_l=1.5
        ppm=l/real_l
        W=px_H/ppm
        depth=W/(2*math.tan(fov_h/2))
        # print(depth)
        return depth




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
                    depth=self.predict_depth(h[0:2,:], v[0:2, :])

                    if center[1]>cross[1] and depth>0:
                        crosses.append((cross, center, depth))
        # print(crosses)
        return crosses


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
