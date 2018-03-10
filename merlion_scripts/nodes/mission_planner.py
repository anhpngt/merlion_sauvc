#!/usr/bin/env python
#Reinaldo Maslim, NTU Merlion 2018

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

class Mission(object):
    #cmd_vel speeds
    forward_speed=1.5
    side_speed=1.5
    yaw_speed=1

    #ODOM
    x0, y0, z0=0, 0, 0
    roll0, pitch0, yaw0=0, 0, 0
    odom_received=False

    #make birdeye heatmap with size 50, 25, ppm=2, init_pos=0.7, 25 
    #3 channels for gate, blue bucket, flare
    heatmaps=np.zeros((50, 100, 3), dtype=np.uint8)
    init_pos=0.7, 25
    ppm=2


    #mission sequence
    seq=[0, 1, 2, 3]

    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        ####Subscribers####
        #sub to heatmaps from detector
        rospy.Subscriber("/detection/heatmap", Image, self.heatmap_callback, queue_size = 1)
        

        #sub to downward cam as main for bucket
        rospy.Subscriber("/down/image_rect_color", Image, self.down_img_callback, queue_size = 1)
        rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.down_img_callback, queue_size = 1)
        

        #sub odom
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)

        while not self.odom_received and not rospy.is_shutdown():
            rospy.sleep(1)
            print("waiting for odom...")


        ####Publishers####
        self.cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.front_img_pub=rospy.Publisher('/mission/front_img', Image, queue_size=1)
        self.down_img_pub=rospy.Publisher('/mission/down_img', Image, queue_size=1)

        for i in self.seq:
            if i==0:
                self.mission_0()
            elif i==1:
                self.mission_1()
            elif i==2:
                self.mission_2()
            elif i==3:
                self.mission_3()

    def mission_0(self, distance=10):
        ####qualification####
        #1.move 10m forward
        #2.move 10m reverse.
        #3.while holding yaw=0 and depth

        print("init qualification task")
        forward_done=False
        

        while not rospy.is_shutdown():
            if self.x0<distance+1 and forward_done==False:
                self.pub_cmd_vel(1, 0, 0)
                print("moving forward")
            else:
                print("moving backward")
                forward_done=True
                self.pub_cmd_vel(-1, 0, 0)
                if self.x0<1:
                    break

        print("qualification done")

    def mission_1(self):
        ####pass through gate####
        #1. look around until confident (yawing or move sideway)
        #2. move sideway directly to front of gate
        #3. move forward 2m
        #4. redo 2 and 3 until passes gate


    def mission_2(self):
        ####drop ball to bucket####
        #1. look around until confident
        #2. move sidewary to front of blue bucket (around red and blue)
        #3. move forward 2m
        #4. redo 2 and 3 until 1 meter behind bucket
        #5. move forward 1m 
        #6. swicth to visual servo via downward cam
        #7. release ball after bucket locked


    def mission_3(self):
        ####hit da flare!!!####
        #1. look around until confident
        #2. yaw to facing flare directly
        #3. move forward 2m
        #4. redo 2 and 3 until passes flare



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

    def pub_cmd_vel(self, vx, vy, vyaw):
        msg=Twist()

        msg.linear.x=vx
        msg.linear.y=vy
        msg.angular.z=vyaw
        self.cmd_vel_pub.publish(msg)

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
    
    def angle_diff(self, minuend, subtrahend): # for angle in [-pi, pi) only!
        diff = minuend - subtrahend
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def confidence(self):
        # Found any objects?
        # TODO
        return 0

    def look_around(self, look_mode):
        original_yaw = self.yaw0
        if look_mode == 'yawing':
            states = ['ccw_yaw', 'cw_yaw', 'return_yaw', 'forward']
            current_state = states[0]
            base_pos_x = self.x0
            base_pos_y = self.y0
            yaw_limit = 45.0 / 180.0 * math.pi
            trl_limit = 2.0 # forward 2m each time

            # Start looking
            while self.confident() < 0.9: ## TODO: (REI) either calling func or global var
                if current_state == states[0]:
                    self.pub_cmd_vel(0, 0, 0.5)
                    if self.angle_diff(z0, original_yaw) >= yaw_limit: # check to switch state
                        current_state = states[1]
                        rospy.loginfo('Switch from {} to {}'.format(states[0], states[1]))
                
                elif current_state == states[1]:
                    self.pub_cmd_vel(0, 0, -0.5)
                    if self.angle_diff(z0, original_yaw) <= -yaw_limit:
                        current_state = states[2]
                        rospy.loginfo('Switch from {} to {}'.format(states[1], states[2]))

                elif current_state == states[2]:
                    self.pub_cmd_vel(0, 0, 0.5)
                    if self.angle_diff(z0, original_yaw) < 0.03 # threshold to go back to original yaw, maybe need something more accurate for this
                        current_state = states[3]
                        rospy.loginfo('Switch from {} to {}'.format(states[2], states[3]))

                elif current_state == states[3]:
                    self.pub_cmd_vel(1, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > trl_limit * trl_limit:
                        current_state = states[0]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[3], states[0]))

        elif look_mode == 'zigzag':
            states = ['left', 'forward', 'right', 'forward']
            current_state = states[0]
            forward_limit = 2.0 
            sideway_limit = 4.0 # move 4m left/right each time
            base_pos_x = self.x0 + sideway_limit / 2.0 * math.cos(original_yaw)
            base_pox_y = self.y0 - sideway_limit / 2.0 * math.cos(original_yaw)

            # Start looking
            while self.confident() < 0.9: # another TODO
                if current_state == states[0]:
                    self.pub_cmd_vel(0, 1, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > sideway_limit * sideway_limit:
                        current_state = states[1]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[0], states[1]))
                
                elif current_state == states[1]:
                    self.pub_cmd_vel(1, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > forward_limit * forward_limit:
                        current_state = states[2]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[1], states[2]))

                elif current_state == states[2]:
                    self.pub_cmd_vel(0, -1, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > sideway_limit * sideway_limit:
                        current_state = states[3]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[2], states[3]))
                
                elif current_state == states[3]:
                    self.pub_cmd_vel(1, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > forward_limit * forward_limit:
                        current_state = states[0]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[3], states[0]))
        
        else:
            rospy.loginfo('Invalid look_around() mode!')
            return
        
        rospy.loginfo('Finished searching!')        
        return

##########################
##########main############
##########################



if __name__ == '__main__':
    try:
        Mission(nodename="mission", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")