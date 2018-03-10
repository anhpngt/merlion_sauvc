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
    x0, y0, z0=0, 0, 1
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

    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        ####Subscribers####

        #front cam gate callback
        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.gate_img_callback, queue_size = 1)
        rospy.Subscriber("/front/image_rect_color", Image, self.gate_img_callback, queue_size = 1)
        #front cam bucket callback
        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.bucket_img_callback, queue_size = 1)
        rospy.Subscriber("/front/image_rect_color", Image, self.bucket_img_callback, queue_size = 1)
        #front cam flare callback
        rospy.Subscriber("/logi_c920/usb_cam_node/image_raw", Image, self.flare_img_callback, queue_size = 1)
        rospy.Subscriber("/front/image_rect_color", Image, self.flare_img_callback, queue_size = 1)

        #sub odom
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)
       while not self.odom_received and not rospy.is_shutdown():
           rospy.sleep(1)
           print("waiting for odom...")


        ####Publishers####
        #pub images for debugging
        self.detection_img_pub=rospy.Publisher('/detection/img', Image, queue_size=1)
        self.birdeye_pub=rospy.Publisher('/detection/birdeye', Image, queue_size=1)

        #pub heatmap of detections with 3 channels for mission planner
        self.heatmap_pub=rospy.Publisher('/detection/heatmap', Image, queue_size=1)

        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            #publish 

            #clean up noise of heatmaps
            # self.heatmaps[self.heatmaps<2]=0


            try:
                #pub detection img
                self.detection_img_pub.publish(self.bridge.cv2_to_imgmsg(self.detection_img, "bgr8"))

                #pub birdeye of combined heatmap
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

            except:
                pass

            rate.sleep()


    def gate_img_callback(self, msg):
        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 255, 0)
        start_time=time.time()
        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # img=self.img_correction(img)
        self.detection_img=img.copy()

        blur = cv2.GaussianBlur(img,(7, 7),0)
        grey=cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                    cv2.THRESH_BINARY,21, 2)
        kernel = np.ones((5,5),np.uint8)    
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        opening=255-opening
        opening = cv2.erode(opening, None, iterations=self.erode)
        
        ###anh's mask###
        # frame_src=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # str_el = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # frame_hsv = cv2.cvtColor(frame_src, cv2.COLOR_BGR2HSV)
        # channels = cv2.split(frame_hsv)
        # frame_binary = cv2.GaussianBlur(channels[2], (5, 5), 0)
        # frame_binary = cv2.adaptiveThreshold(frame_binary, 255, 1, 1, 51, 3)
        # frame_result = cv2.morphologyEx(frame_binary, cv2.MORPH_CLOSE, str_el)
        # opening = cv2.morphologyEx(frame_result, cv2.MORPH_OPEN, str_el)

        minLineLength=70
        lines = cv2.HoughLinesP(image=opening,rho=1,theta=np.pi/180,\
         threshold=80,lines=np.array([]), minLineLength=minLineLength,maxLineGap=12)

        heatmap=np.zeros_like(opening)
        side=10
        try:
            h_lines=[]
            v_lines=[]

            if lines is None:
                self.erode=1

            for line in lines:
                
                x1, y1, x2, y2=line[0][0], line[0][1], line[0][2], line[0][3]
                
                theta=abs(math.atan(float(y2-y1)/(x2-x1+0.001))*180/math.pi)
                # print(theta)
                angle_thres=30
                if theta<angle_thres:
                    #horizontal
                    # cv2.line(self.detection_img, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                    h_lines.append(np.array([[x1, y1], [x2, y2]]))
                elif abs(theta-90)<angle_thres:
                    #vertical
                    # cv2.line(self.detection_img, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)
                    v_lines.append(np.array([[x1, y1], [x2, y2]]))


            depths=[]
            if len(h_lines)>0 and len(v_lines)>0:
                crosses=self.find_crosses(h_lines, v_lines)
                
                for cross, center, depth in crosses:
                    depths.append(depth)
                    heatmap[center[1]-side:center[1]+side, center[0]-side:center[0]+side]+=1
                    text="distance: "+str(round(depth, 2)) +"m"
                    # cv2.putText(self.detection_img, text, (int(cross[0])+10, int(cross[1])-20), font, 0.5, color, 1, cv2.LINE_AA)


            if len(depths)==0:
                self.gate_rect=None
                return
            hist, bin_edges = np.histogram(np.asarray(depths), bins='fd', density=False)
            ind=np.argmax(hist)
            depth=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)
            
            #if height too low, only detect tiles
            if self.z0<0.5:
                self.gate_rect=None
                return

            #process heatmap
            gate=(np.argmax(heatmap)%img.shape[1], int(math.floor(np.argmax(heatmap)/img.shape[1])+1))
            cv2.circle(self.detection_img, (int(gate[0]), int(gate[1])), 20, (30, 200, 0), -1)


            H=depth*(2*math.tan(self.fov_h/2))
            ppm=self.px_H/H
            real_l=1.5
            l=real_l*ppm

            cv2.rectangle(self.detection_img, (int(gate[0]-l/2),int(gate[1]-l/2)), \
                (int(gate[0]+l/2),int(gate[1]+l/2)), (0, 255, 0), 2)

            max_val=np.amax(heatmap)
            if max_val>0:
                heatmap_img = cv2.applyColorMap(heatmap*int(255/max_val), cv2.COLORMAP_JET)
            else:
                heatmap_img = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)

            #compute real position of gate in x,y,z
            #first x and y refers to side and height


            x, y, z=self.compute_xy(gate[0], gate[1], depth, img)
            
            #filter out weird locations
            #width of pool on each side, pool y direction
            pool_w=15
            #located around 8 meters x
            if abs(x-8)>2 and pool_w-abs(y)<0:
                #out of expected area
                return


            cv2.putText(self.detection_img, str(z), (50, 50), font, 0.5, color, 1, cv2.LINE_AA)
            #plot in map
            ind_x=int(self.heatmaps.shape[0]-(self.init_pos[0]+x)*self.ppm)
            ind_y=int((self.init_pos[1]-y)*self.ppm)
            if ind_x>self.heatmaps.shape[0]-1 or ind_y>self.heatmaps.shape[1]-1:
                return
            self.heatmaps[ind_x, ind_y, 0]+=1

        except:
            pass


        # frame_binary = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR);
        # cv2.imshow('Image', frame_binary)
        # cv2.waitKey(3)

    def bucket_img_callback(self, msg):

        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)
        start_time=time.time()

        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img=self.img_correction(img)

        h, w = img.shape[:2]            
        output=np.zeros_like(img)
        blur = cv2.GaussianBlur(img,(7, 7),0)
        #red then blue
        boundaries = [
            ([0, 0, 0], [125, 105, 255], [0, 0, 255]),
            ([0, 0, 0], [255, 128, 50], [255, 0, 0])
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
                if self.z0<0.5:
                    break

                rect = cv2.boundingRect(contour)
                area=int(rect[3]*rect[2])
                ar=float(rect[3])/rect[2]
                # print(ar)
                px_count=np.sum(opening[rect[1]:rect[1]+rect[3], rect[0]:rect[0]+rect[2]])/255
                # print(self.px_count/area)
                if ar<1 and ar>0.5 and area>1000 and area<120000 and px_count/area>0.8 and rect[1]>h/2:
                    # print(self.px_count/area)
                    cv2.rectangle(self.detection_img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)

                    depth=self.predict_depth_bucket(max(rect[2], rect[3]))
                    x, y, z=self.compute_xy(rect[0]+rect[2]/2, rect[1]+rect[3]/2, depth, img)
                    text="x, y: "+str(round(x, 2)) +"m "+str(round(y, 2))+"m"
                    cv2.putText(self.detection_img, text, (int(rect[0])+10, int(rect[1])-20), font, 0.5, color, 1, cv2.LINE_AA)

                    #update heatmap
                    ind_x=int(self.heatmaps.shape[0]-(self.init_pos[0]+x)*self.ppm)
                    ind_y=int((self.init_pos[1]-y)*self.ppm)

                    if ind_x>self.heatmaps.shape[0]-1 or ind_y>self.heatmaps.shape[1]-1:
                        return

                    if i==1:
                        #blue bucket heatmap
                        self.heatmaps[ind_x, ind_y, 1]+=1
                i+=1

        # frame_binary = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR);
        # cv2.imshow('Image', frame_binary)
        # cv2.waitKey(3)

    def flare_img_callback(self, msg):
        font = cv2.FONT_HERSHEY_SIMPLEX
        color=[0, 255, 255]
        str_el = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        frame_src=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # frame_src=self.img_correction(frame_src)

        if self.z0<0.5:
            return

        frame_hsv = cv2.cvtColor(frame_src, cv2.COLOR_BGR2HSV)
        channels = cv2.split(frame_hsv)
        frame_binary = cv2.GaussianBlur(channels[1], (5, 5), 0)
        frame_binary = cv2.adaptiveThreshold(frame_binary, 255, 1, 1, 21, 3)
        frame_binary = cv2.morphologyEx(frame_binary, cv2.MORPH_CLOSE, str_el)
        frame_binary = cv2.morphologyEx(frame_binary, cv2.MORPH_OPEN, str_el)

        # cv2.imshow('Image', frame_binary)
        # cv2.waitKey(3)
        _, contours, _ = cv2.findContours(frame_binary.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contours
        best_detection = 0
        max_area = 0.0
        best_rect=None
        for i in range(len(contours)):
            contour_area = cv2.contourArea(contours[i])
            if contour_area < 300:
                continue


            hull = cv2.convexHull(contours[i], clockwise=0, returnPoints=1)
            rect = cv2.boundingRect(contours[i])
            rotatedrect = cv2.minAreaRect(contours[i])
            minrect = cv2.boxPoints(rotatedrect)

            mr_area = cv2.contourArea(minrect)
            hull_area = cv2.contourArea(hull)

            # Condition:
            #   area check: area of contour must be close to the min_area_rect bounded
            #   area check: similar, but between hull_area and min_area_rect bounded
            #   height check: object's height must be considerable compare to image height
            #   aspect ratio check: height >> width
            if float(contour_area) / mr_area > 0.4 and float(hull_area) / mr_area > 0.7 and rect[3] * 4 > len(frame_src) and float(rect[3]) / rect[2] > 3:
                # If found a bigger detection, register
                if contour_area > max_area:
                    # print('Area: {}, mr_area: {}, hull_area: {}, height: {}'.format(contour_area, mr_area, hull_area, rect[3]))
                    best_detection = i
                    max_area = contour_area
                    best_rect=rect

        if best_rect is None:
            return
            
        rect=best_rect

        # After processing all contours, if found a detection, visualize and send cmd_vel
        if max_area > 150:
            # cv2.drawContours(self.detection_img, contours, best_detection, color, 2)
            cv2.rectangle(self.detection_img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)
        # frame_binary = cv2.cvtColor(frame_binary, cv2.COLOR_GRAY2BGR);
        print(float(rect[3])/rect[2])
        
        l=math.sqrt(rect[2]**2+rect[3]**2)
        depth=self.predict_depth_flare(l)
        x, y, z=self.compute_xy(rect[0]+rect[2]/2, rect[1]+rect[3]/2, depth, frame_src)

        text="x, y: "+str(round(x, 2)) +"m "+str(round(y, 2))+"m"
        cv2.putText(self.detection_img, text, (int(rect[0])+10, int(rect[1])-20), font, 0.5, color, 1, cv2.LINE_AA)

        #update heatmap
        ind_x=int(self.heatmaps.shape[0]-(self.init_pos[0]+x)*self.ppm)
        ind_y=int((self.init_pos[1]-y)*self.ppm)

        if ind_x>self.heatmaps.shape[0]-1 or ind_y>self.heatmaps.shape[1]-1:
            return

        #flare heatmap
        self.heatmaps[ind_x, ind_y, 2]+=1


    def compute_xy(self, px, py, depth, img):
        #compute real position of gate in x,y,z
        #first x and y refers to side and height, depth is in cam frame

        #depth in pixel
        pd=(self.px_W/2)/math.tan(self.fov_w/2)
        
        #cam x and y deviation in pixels
        del_px=(-px+img.shape[1]/2.0)
        del_py=(-py+img.shape[0]/2.0)

        #cam x and y deviation in meters
        del_x=depth*del_px/pd
        del_y=depth*del_py/pd

        #compensated for rolling and pitch
        del_real_x=del_x*math.cos(self.roll0)-del_y*math.sin(self.roll0)
        del_real_y=del_x*math.sin(self.roll0)+del_y*math.cos(self.roll0)-depth*math.tan(self.pitch0)
        
        #global position with odom
        x=self.x0+depth*math.cos(self.yaw0)-del_real_x*math.sin(self.yaw0)
        y=self.y0+depth*math.sin(self.yaw0)+del_real_x*math.cos(self.yaw0)
        z=self.z0+del_real_y
            
        return x, y, z
        
    def predict_depth_bucket(self, l):

        #the longest edge is independent of perspective
        #real length of pole in metres
        real_l=0.55
        ppm=l/real_l
        H=self.px_H/ppm
        depth=H/(2*math.tan(self.fov_h/2))
        # print(depth)
        return depth

    def predict_depth_flare(self, l):

        #the longest edge is independent of perspective
        #real length of pole in metres
        real_l=1.5
        ppm=l/real_l
        H=self.px_H/ppm
        depth=H/(2*math.tan(self.fov_h/2))
        # print(depth)
        return depth

    def predict_depth_gate(self, line1, line2):

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
        H=self.px_H/ppm
        depth=H/(2*math.tan(self.fov_h/2))

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
                    depth=self.predict_depth_gate(h[0:2,:], v[0:2, :])

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


    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.z0 = msg.pose.pose.position.z
        # print(self.z0)
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        

##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        Detector(nodename="detector", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
