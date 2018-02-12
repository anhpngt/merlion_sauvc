#!/usr/bin/env python

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

class DetectBucket(object):

    x0, y0, z0=0, 0, 0
    roll0, pitch0, yaw0=0, 0, 0
    odom_received=False

    #make birdeye heatmap with size 50, 25, ppm=2, init_pos=0.7, 25 
    birdeye_heatmap=np.zeros((50, 100), dtype=np.uint8)
    init_pos=0.7, 25
    ppm=2

    heatmaps=np.zeros((2, 50, 100), dtype=np.uint8)



    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()
        self.init_markers()

        #sub to front cam
        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 1)
        rospy.Subscriber("/front/image_rect_color", Image, self.img_callback, queue_size = 1)

        #sub to downward cam
        rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.down_img_callback, queue_size = 1)
        rospy.Subscriber("/down/image_rect_color", Image, self.down_img_callback, queue_size = 1)

        #sub to odom
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)

        #publish detection imgs
        self.img_pub=rospy.Publisher('/bucket_img', Image, queue_size=1)
        self.down_img_pub=rospy.Publisher('/down_bucket_img', Image, queue_size=1)
        self.birdeye_heatmap_pub=rospy.Publisher('/birdeye_bucket', Image, queue_size=1)



        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


    def down_img_callback(self, msg):


        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)

        fov_w, fov_h=48*math.pi/180, 36*math.pi/180
        px_W, px_H=640, 480


        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img=self.img_correction(img)

        h, w = img.shape[:2]            
        output=np.zeros_like(img)
        blur = cv2.GaussianBlur(img,(7, 7),0)

        boundaries = [
            ([90, 90, 100], [155, 145, 255], [0, 0, 255]),
            ([50, 31, 4], [255, 128, 50], [255, 0, 0])
            # ([25, 146, 190], [62, 174, 250]),
            # ([103, 86, 65], [145, 133, 128])
        ]   

        combined_mask=np.zeros((h, w), dtype=np.uint8)
        i=0
        for (lower, upper, color) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper)
            
            # minLineLength=300
            # lines = cv2.HoughLinesP(image=mask,rho=1,theta=np.pi/180,\
            #  threshold=50,lines=np.array([]), minLineLength=minLineLength, maxLineGap=12)
            # if lines is not None:
            #     for line in lines:
            #         #find two major gradients            
            #         x1, y1, x2, y2=line[0][0], line[0][1], line[0][2], line[0][3]
                    
            #         cv2.line(mask, (x1, y1), (x2, y2), 0, 5, cv2.LINE_AA)

            kernel = np.ones((5,5),np.uint8)    
            contour_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            # opening=mask
            # opening = cv2.erode(mask, None, iterations=4)
            opening = cv2.dilate(mask, None, iterations=2)
            # Hough circle function parameters
            (_,contours,_) = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contour_mask=cv2.cvtColor(contour_mask, cv2.COLOR_GRAY2BGR)            
            areas=[]
            r=[]

            for contour in contours:
                rect = cv2.boundingRect(contour)
                area=rect[2]*rect[3]
                ar=float(rect[3])/rect[2]
                # print(ar)
                px_count=np.sum(opening[rect[1]:rect[1]+rect[3], rect[0]:rect[0]+rect[2]])/255
                # print(px_count/area)
                if ar<1.5 and ar>0.6 and area>10000 and px_count/area>0.5:
                    cv2.rectangle(img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)
                    del_px=w/2-(rect[0]+rect[2]/2)
                    del_py=h/2-(rect[1]+rect[3]/2)
                    thetax=fov_w*del_px/(w/2)
                    thetay=fov_h*del_py/(h/2)
                    
                    del_x=self.z0*math.tan(thetax)
                    del_y=self.z0*math.tan(thetay)
                    x=self.x0-del_y
                    y=self.y0-del_x

                    ind_x=int(self.birdeye_heatmap.shape[0]-(self.init_pos[0]+x)*self.ppm)
                    ind_y=int((self.init_pos[1]-y)*self.ppm)

                    ind_x=np.clip(ind_x, 0, self.birdeye_heatmap.shape[1])
                    ind_y=np.clip(ind_y, 0, self.birdeye_heatmap.shape[0])
                    self.heatmaps[i, ind_x, ind_y]+=1                        



            combined_mask=cv2.bitwise_or(combined_mask, mask)
            i+=1

        combined_mask=cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        
        self.down_img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, combined_mask]), "bgr8"))






    def img_callback(self, msg):

        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)
        start_time=time.time()

        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img=self.img_correction(img)

        h, w = img.shape[:2]            
        output=np.zeros_like(img)
        blur = cv2.GaussianBlur(img,(7, 7),0)
        #red, then blue
        boundaries = [
            # ([0, 0, 0], [125, 105, 255], [0, 0, 255]),
            ([0, 31, 4], [255, 128, 50], [255, 0, 0])
            # ([25, 146, 190], [62, 174, 250]),
            # ([103, 86, 65], [145, 133, 128])
        ]   

        combined_mask=np.zeros((h, w), dtype=np.uint8)
        

        i=0
        for (lower, upper, color) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper)
            
            kernel = np.ones((5,5),np.uint8)    
            # opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            # opening=mask
            opening = cv2.erode(mask, None, iterations=4)
            # opening = cv2.dilate(mask, None, iterations=2)
            (_,contours,_) = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                        
            for contour in contours:
                #if too low not valid
                if self.z0<0.7:
                    break

                rect = cv2.boundingRect(contour)
                area=int(rect[3]*rect[2])
                ar=float(rect[3])/rect[2]
                # print(ar)
                px_count=np.sum(opening[rect[1]:rect[1]+rect[3], rect[0]:rect[0]+rect[2]])/255
                # print(px_count/area)
                if ar<1 and ar>0.5 and area>1000 and area<120000 and px_count/area>0.5:
                    # print(px_count/area)
                    cv2.rectangle(img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)

                    depth=self.predict_depth(max(rect[2], rect[3]))
                    x, y=self.compute_xy(rect[0]+rect[2]/2, rect[1]+rect[3]/2, depth, img)
                    print(depth, x, y)
                    text="x, y: "+str(round(x, 2)) +"m "+str(round(y, 2))+"m"
                    cv2.putText(img, text, (int(rect[0])+10, int(rect[1])-20), font, 0.5, color, 1, cv2.LINE_AA)

                    #update heatmap
                    ind_x=int(self.birdeye_heatmap.shape[0]-(self.init_pos[0]+x)*self.ppm)
                    ind_y=int((self.init_pos[1]-y)*self.ppm)

                    ind_x=np.clip(ind_x, 0, self.birdeye_heatmap.shape[1])
                    ind_y=np.clip(ind_y, 0, self.birdeye_heatmap.shape[0])


                    # self.heatmaps[i, ind_x, ind_y]+=1

            #threshold->cluster heatmap to get members




            combined_mask=cv2.bitwise_or(combined_mask, mask)
            i+=1

        self.birdeye_heatmap=np.sum(self.heatmaps, axis=0).astype(np.uint8)




        max_val=10 #np.amax(self.birdeye_heatmap)

        self.birdeye_heatmap=np.clip(self.birdeye_heatmap, 0, max_val)
        if max_val>0:
            birdeye_heatmap_img = cv2.applyColorMap(self.birdeye_heatmap*int(255/max_val), cv2.COLORMAP_JET)
        else:
            birdeye_heatmap_img = cv2.applyColorMap(self.birdeye_heatmap, cv2.COLORMAP_JET)        
        
        self.birdeye_heatmap_pub.publish(self.bridge.cv2_to_imgmsg(birdeye_heatmap_img, "bgr8"))

        combined_mask=cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, combined_mask]), "bgr8"))


    def compute_xy(self, px, py, depth, img):
        #compute real position of gate in x,y,z
        #first x and y refers to side and height
        fov_w, fov_h=62*math.pi/180, 46*math.pi/180
        px_W, px_H=640, 480

        pd=(px_W/2)/math.tan(fov_w/2)
        del_px=(-px+img.shape[1]/2.0)
        del_py=(-py+img.shape[0]/2.0)
        del_x=depth*del_px/pd
        del_y=depth*del_py/pd

        del_real_x=del_x*math.cos(self.roll0)-del_y*math.sin(self.roll0)
        del_real_y=del_x*math.sin(self.roll0)+del_y*math.cos(self.roll0)+depth*math.tan(self.pitch0)
        
        x=self.x0+depth*math.cos(self.yaw0)-del_real_x*math.sin(self.yaw0)
        y=self.y0+depth*math.sin(self.yaw0)+del_real_x*math.cos(self.yaw0)
        z=self.z0+del_real_y
        
        return x, y


    def predict_depth(self, l):
        fov_w, fov_h=62*math.pi/180, 46*math.pi/180
        px_W, px_H=640, 480
        #the longest edge is independent of perspective
        #real length of pole in metres
        real_l=0.55
        ppm=l/real_l
        H=px_H/ppm
        depth=H/(2*math.tan(fov_h/2))
        # print(depth)
        return depth


    def img_correction(self, img):
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        res=np.zeros_like(img)
        for i in range(3):
            res[:, :, i] = clahe.apply(img[:, :, i])
        return res


    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.z0 = msg.pose.pose.position.z
        # print(self.z0)
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        


    def printMarker(self, gate_pos):
                #markerList store points wrt 2D world coordinate
                
        self.markers.points=[]
        p=Point()

        self.markers.points.append(Point(0, 0, 0))
        p.x=gate_pos[0]
        p.y=gate_pos[1]
        p.z=0.75
        q_angle = quaternion_from_euler(0, 0, 0)
        q = Quaternion(*q_angle)
        self.markers.pose = Pose(p, q)

        self.marker_pub.publish(self.markers)


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'frontiers'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('gate_markers', Marker, queue_size=5)

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

##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        DetectBucket(nodename="detect_bucket", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
