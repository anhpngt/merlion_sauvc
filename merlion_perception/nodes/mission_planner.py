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
from std_msgs.msg import Bool
import time
import random

#################################
##############class##############
#################################

class Mission(object):
    # sleeping time
    timestep = 0.1

    #cmd_vel speeds, in m/s and rad/s
    forward_speed = 0.8
    side_speed = 0.2
    yaw_speed = 0*math.pi/180

    thres=0.5

    #ODOM
    x0, y0, z0 = 0, 0, 0
    roll0, pitch0, yaw0 = 0, 0, 0
    odom_received = False

    #make birdeye heatmap with size 50, 25, ppm=2, init_pos=0.7, 25 
    #3 channels for gate, blue bucket, flare
    heatmaps = np.zeros((50, 100, 3), dtype=np.uint8)
    init_pos = 0.7, 25
    ppm = 2

    #mission sequence
    seq = [1, 3, 2]
    
    #stores detections, row wise: gate, bucket, flare, col wise: x, y, confidence
    detections = np.zeros((3, 3))

    #visual servoing params
    blue_del_x, blue_del_y, streak = 0, 0, 0
    bucket_seen = False

    #look around bias, estimated global position of gate, bucket, and flare
    #detection_bias=[[0, 0], [0, 0], [0, 0]]
    detection_bias = [[6, -1], [23, 2], [12, -6]]

    #release ball on mission 2
    drop_ball=False
    streak=0

    #0=red, blue=1
    bucket_color=1

    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        ####Subscribers####

        #sub to heatmaps from detector
        #if len(self.seq)==1 and self.seq[0]==0:
        #    print("qualifier don't care localizer")
        #else:
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)
        while not self.odom_received and not rospy.is_shutdown():
           rospy.sleep(1)
           rospy.loginfo("Waiting for odom...")

        #sub to heatmaps from detector
        rospy.Subscriber("/detection/heatmap", Image, self.heatmap_callback, queue_size = 1)
        
        #sub to downward cam as main for bucket
        rospy.Subscriber("/down/image_rect_color", Image, self.down_img_callback, queue_size = 1)

        ####Publishers####
        self.cmd_vel_pub = rospy.Publisher('/merlion/control/cmd_vel', Twist, queue_size=1)
        self.front_img_pub = rospy.Publisher('/mission/front_img', Image, queue_size=1)
        self.down_img_pub = rospy.Publisher('/mission/down_img', Image, queue_size=1)
        self.drop_ball_pub= rospy.Publisher('/merlion/drop_ball', Bool, queue_size=1)

        for i in self.seq:
            if i == 0:
                self.mission_0()
            elif i == 1:
                self.mission_1()
            elif i == 2:
                self.mission_2()
            elif i == 3:
                self.mission_3()

    def mission_0(self):
        ####qualification####
        #1.move 10m forward
        #2.move 10m reverse.
        #3.while holding yaw=0 and depth

        rospy.loginfo("Attempting qualification task...")
        while not rospy.is_shutdown():
            self.pub_cmd_vel(self.forward_speed, 0, 0)

        rospy.loginfo("qualification done")

    def mission_1(self):
        ####pass through gate####
        #1. look around until confident (yawing or move sideway)
        #2. move sideway directly to front of gate
        #3. move forward 2m
        #4. redo 2 and 3 until passes gate
        #########################
        rospy.loginfo("init mission 1")

        #step 1 
        self.look_around(1, 15)

        #distance threshold in sideway movement

        #move this much pass the gate
        offset=2
        #step 4 enclosing step 2 and step 3
        while not rospy.is_shutdown():
            x, y, conf=self.detections[0]
            print(x, y)
            #step 2
            error_y=y-self.y0
            if abs(error_y)>self.thres:
                rospy.loginfo("1.2 move sideway")
                sign=np.sign(error_y)
                self.pub_cmd_vel(0, sign*self.side_speed,0)
            else:
                #step 3
                rospy.loginfo("1.3 move forward")
                if x+offset-self.x0>0:
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                else:
                    #bravo we've passed the gate!!!
                    break

        rospy.loginfo("mission 1 success")

    def mission_2(self):
        ####drop ball to bucket####
        #1. look around until confident
        #2. move sidewary to front of blue bucket (around red and blue)
        #3. move forward 2m
        #4. redo 2 and 3 until 1 or 2 meters behind bucket
        #5. move forward 1m without localization(via time) or 
        #6. swicth to visual servo via downward cam
        #7. release ball after bucket locked
        #8. reverse 3m
        ###########################
        rospy.loginfo("init mission 2")
        
        offset=-2
        #step 1

        self.look_around(2, 10)
        
        #step 4 enclosing 2 and 3
        while not rospy.is_shutdown():
            x, y, conf=self.detections[1]

            #step 2
            error_y=y-self.y0
            if abs(error_y)>self.thres:
                rospy.loginfo("2.2 moving sideway")
                sign=np.sign(error_y)
                self.pub_cmd_vel(0, sign*self.side_speed,0)
            else:
                #step 3
                
                if x+offset-self.x0>0:
                    rospy.loginfo("2.3 move forward")
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                else:
                    #bravo we're near the bucket
                    break 

        #step 5

        #set few timesteps to foward amount of 2 m ##TODO tune ts
        ts=3
        for i in range(int(ts/self.timestep)):
            rospy.loginfo("blind motion")
            self.pub_cmd_vel(self.forward_speed, 0, 0)
            if rospy.is_shutdown():
                return
            
        #step 6
        sign=0
        while not rospy.is_shutdown():   
            #pixel to meter
            k=self.forward_speed/320

            if self.bucket_seen is True:
                rospy.loginfo("2.6 visual servo adjusting to bucket")
                #body x axis is in image +y direction
                #body y axis is in image +x direction
                vs_x=self.blue_del_y*k
                vs_y=self.blue_del_x*k
                self.pub_cmd_vel(vs_x, vs_y, 0)
                print(vs_x, vs_y, self.streak)

                if self.streak>4:
                    break
            else:
                rospy.loginfo("2.6 visual servo random search left&right")
                #search to left or right
                #random direction to go amount of #2m
                
                if sign==0:
                    ts=1
                elif sign==1:
                    ts*=2
                for i in range(int(ts/self.timestep)):
                    self.pub_cmd_vel(0, self.side_speed*(-1)**sign, 0)
                    if rospy.is_shutdown():
                        return
                sign+=1


        #step 7
        rospy.loginfo("2.7 drop ball")
 

        #step 8
        #set few timesteps to reverse
        ts=5
        for i in range(int(ts/self.timestep)):
            rospy.loginfo("blind motion")
            self.pub_cmd_vel(-self.forward_speed, 0, 0)
            if rospy.is_shutdown():
                return
       
        rospy.loginfo("mission 2 success")

    def mission_3(self):
        ####hit da flare!!!####
        #1. look around until confident
        #2. move sideway until facing flare
        #3. move forward 2m
        #4. redo 2 and 3 until passes flare
        ######################
        rospy.loginfo("init mission 3")

        #step 1
        self.look_around(3, 15)
        
        #step 4 enclosing 2 and 3
        while not rospy.is_shutdown():            
            x, y, conf=self.detections[2]

            #step 2
            error_y=y-self.y0
            if abs(error_y)>self.thres:
                rospy.loginfo("3.2 move sideway")
                sign=np.sign(error_y)
                self.pub_cmd_vel(0, sign*self.side_speed,0)
            else:
                #step 3
                rospy.loginfo("3.3 move forward")
                self.pub_cmd_vel(self.forward_speed, 0, 0)
                if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<self.thres:
                    break
        #step 
        #set few timesteps to foward
        ts=3
        for i in range(int(ts/self.timestep)):
            self.pub_cmd_vel(self.forward_speed, 0, 0)

        rospy.loginfo("mission 3 success")


    def look_around(self, mission_no, conf_thres=10):
        txt=str(mission_no)+".1 lookaround"
        i=mission_no-1
        rospy.loginfo(txt)

        bias=self.detection_bias[mission_no-1]
        rospy.loginfo(bias)

        if bias[0]!=0 or bias[1]!=0:
            r=3
            #go to proximity of bias
            x=bias[0]
            y=bias[1]

            _, _, conf=self.detections[i]
            while conf < conf_thres and not rospy.is_shutdown():            
                
                error_y=y-self.y0
                if abs(error_y)>self.thres:
                    rospy.loginfo("lookaround sideway")
                    sign=np.sign(error_y)
                    print(error_y)
                    self.pub_cmd_vel(0, sign*self.side_speed,0)
                else:
                    #step 3
                    # rospy.loginfo("forward towards bias")
                    self.pub_cmd_vel(self.forward_speed, 0, 0)

                    if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<r:
                        break
                
                _, _, conf=self.detections[i]

        rospy.loginfo('Finished searching!')        
        return



    def down_img_callback(self, msg):
        #visual servoing, assume in frame only 1 bucket of each color at a time
        #go to blue as priority

        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)

        fov_w, fov_h=48*math.pi/180, 36*math.pi/180
        px_W, px_H=640, 480

        img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img=self.img_correction(img)

        h, w = img.shape[:2]            
        output=np.zeros_like(img)
        blur = cv2.GaussianBlur(img,(7, 7),0)

        #red then blue
        boundaries = [
            ([50, 50, 100], [155, 155, 255], [0, 0, 255]),
            ([150, 50, 0], [255, 150, 100], [255, 0, 0])
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
                # rospy.loginfo(ar)
                px_count=np.sum(opening[rect[1]:rect[1]+rect[3], rect[0]:rect[0]+rect[2]])/255
                # rospy.loginfo(px_count/area)
                if ar<1.5 and ar>0.6 and area>10000 and px_count/area>0.5:
                    self.bucket_seen=True
                    cv2.rectangle(img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)
                    del_x=(rect[0]+rect[2]/2)-w/2
                    del_y=(rect[1]+rect[3]/2)-h/2
                    if i!=self.bucket_color:
                        print("unwanted bucket")
                    elif i==self.bucket_color:
                        print("wanted bucket")
                        self.blue_del_x=del_x
                        self.blue_del_y=del_y

                        #if sub inside bucket
                        if rect[0]<w/2 and rect[2]+rect[0]>w/2 and rect[1]<h/2 and rect[1]+rect[3]>h/2:
                            self.streak+=1
			    if self.streak>4:
				self.release_ball()
                        else:
                            self.streak=0
                    print(del_x, del_y)
                    # rospy.loginfo(self.bucket_seen, self.del_x, self.del_y)

            combined_mask=cv2.bitwise_or(combined_mask, mask)
            i+=1

        combined_mask=cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        
        self.down_img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, combined_mask]), "bgr8"))

    def release_ball(self):
        msg=Bool()

        for i in range(int(10/self.timestep)):
            msg.data=True
            self.drop_ball_pub.publish(msg)
            rospy.sleep(self.timestep)

        rospy.loginfo("ball released")


    def pub_cmd_vel(self, vx, vy, vyaw):
        msg=Twist()

        msg.linear.x=vx
        msg.linear.y=vy
        msg.angular.z=vyaw
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(self.timestep)


    def heatmap_callback(self, msg):
        heatmap=self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #find each channel's max and confidence
        for i in range(3):
            ind=np.argmax(heatmap[:, :, i])
            ind_x=int(ind/heatmap.shape[1])
            ind_y=ind%heatmap.shape[1]

            #convert to global map
            x=(heatmap.shape[0]-ind_x)/self.ppm-self.init_pos[0]
            y=self.init_pos[1]-ind_y/self.ppm

            conf=np.amax(heatmap[:, :, i])

            self.detections[i]=np.array([x, y, conf])


    def angle_diff(self, minuend, subtrahend): 
        diff = minuend - subtrahend
        return math.atan2(math.sin(diff), math.cos(diff))


    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.z0 = msg.pose.pose.position.z
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        

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

    try:
        Mission(nodename="mission", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
