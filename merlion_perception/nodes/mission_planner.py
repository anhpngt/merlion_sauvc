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

    #stores detections, row wise: gate, bucket, flare, col wise: x, y, confidence
    detections=np.zeros((3, 3))

    #visual servoing params
    del_x, del_y=0, 0
    bucket_seen=False


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
                self.pub_cmd_vel(self.forward_speed, 0, 0)
                # print("moving forward")
            else:
                # print("moving backward")
                forward_done=True
                self.pub_cmd_vel(-self.forward_speed, 0, 0)
                if self.x0<1:
                    break

        print("qualification done")

    def mission_1(self):
        ####pass through gate####
        #1. look around until confident (yawing or move sideway)
        #2. move sideway directly to front of gate
        #3. move forward 2m
        #4. redo 2 and 3 until passes gate
        #########################
        print("init mission 1")

        #step 1 
        self.look_around(0, 'yawing', 20)

        #distance threshold in sideway movement
        thres=0.5

        #step 4 enclosing step 2 and step 3
        while not rospy.is_shutdown():
            x, y, conf=self.detections[0]

            #step 2
            error_y=y-self.y0
            if abs(error_y)>thres:
                sign=np.sign(error_y)
                self.pub_cmd_vel(0, sign*self.side_speed,0)
            else:
                #step 3
                if x-self.x0>0:
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                else:
                    #bravo we've passed the gate!!!
                    break

        print("mission 1 success")

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
        #9. rotate 
        ###########################
        print("init mission 2")

        #step 1
        self.look_around(1, 'yawing', 20)

        #step 4 enclosing 2 and 3
        while not rospy.is_shutdown():
            x, y, conf=self.detections[1]

            #step 2
            error_y=y-self.y0
            if abs(error_y)>thres:
                sign=np.sign(error_y)
                self.pub_cmd_vel(0, sign*self.side_speed,0)
            else:
                #step 3
                dist=1
                if x-dist-self.x0>0:
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                else:
                    #bravo we're near the bucket
                    break 

        #step 5
        #tell motor controller to switch to blind mode
        self.blind_mode()
        #set few timesteps to foward
        ts=3
        for i in range(int(ts/0.1)):
            print("blind motion")
            self.pub_cmd_vel(self.forward_speed, 0, 0)
            rospy.sleep(0.1)


        #step 6
        while not rospy.is_shutdown():   
            k=self.forward_speed/320

            if self.bucket_seen is True:
                vs_x=self.del_x*k
                vs_y=self.del_y*k
                self.pub_cmd_vel(vs_x, vs_y, 0)

                if abs(vs_x)<0.2 and abs(vs_y)<0.2:
                    break
            else:
                #search to left or right
                #random direction to go
                ts=3
                for i in range(int(ts/0.1)):
                    seed=random.randint(0, 10)%2
                    self.pub_cmd_vel(0, self.side_speed*(-1)**seed, 0)
                    rospy.sleep(0.1)

        #step 7
        self.release_ball()
        rospy.sleep(3)

        #step 8
        #set few timesteps to foward
        ts=3
        for i in range(int(ts/0.1)):
            print("blind motion")
            self.pub_cmd_vel(-self.forward_speed, 0, 0)
            rospy.sleep(0.1)
        #switch back to localizer mode
        self.blind_mode()


        #step 9
        #rotate facing -180/180 deg
        yaw_des=math.pi
        while not rospy.is_shutdown():            
            #step 2
            error_yaw=yaw_des-self.yaw0
            error_yaw=math.atan2(math.sin(error_yaw), math.cos(error_yaw))
            ang_thres=10*math.pi/180

            if abs(error_yaw)>ang_thres:
                sign=np.sign(error_yaw)
                self.pub_cmd_vel(0, 0, sign*self.yaw_speed)

        print("mission 2 success")

    def mission_3(self):
        ####hit da flare!!!####
        #1. look around until confident
        #2. yaw to facing flare directly
        #3. move forward 2m
        #4. redo 2 and 3 until passes flare
        ######################
        print("init mission 3")

        #step 1
        self.look_around(2, 'zigzag', 20)
        
        #step 4 enclosing 2 and 3
        while not rospy.is_shutdown():            
            x, y, conf=self.detections[2]

            #step 2
            yaw_des=math.atan2(x-self.x0, y-self.y0)
            error_yaw=yaw_des-self.yaw0
            error_yaw=math.atan2(math.sin(error_yaw), math.cos(error_yaw))
            ang_thres=10*math.pi/180

            error_dis=math.sqrt((x-self.x0)**2+(y-self.y0)**2)
            dis_thres=1

            if abs(error_yaw)>ang_thres and error_dis>dis_thres:
                sign=np.sign(error_yaw)
                
                last_region_sign=[np.sign(x-self.x0), np.sign(y-self.y0)]

                self.pub_cmd_vel(0, 0, sign*self.yaw_speed)

            else:
                #step 3
                self.pub_cmd_vel(self.forward_speed, 0, 0)

                region_sign=[np.sign(x-self.x0), np.sign(y-self.y0)]
                if region_sign[0]*last_region_sign[0]==-1 and region_sign[1]*last_region_sign[1]==-1:
                    break

        print("mission 3 success")



    def angle_diff(self, minuend, subtrahend): 
        diff = minuend - subtrahend
        return math.atan2(math.sin(diff), math.cos(diff))

    def look_around(self, i, mode, conf_thres=20):
        original_yaw = self.yaw0

        if mode == 'yawing':
            states = ['ccw_yaw', 'cw_yaw', 'return_yaw', 'forward']
            current_state = states[0]
            base_pos_x = self.x0
            base_pos_y = self.y0
            yaw_limit = 45.0 / 180.0 * math.pi
            trl_limit = 2.0 # forward 2m each time


            _, _, conf=self.detections[i]
            # Start looking
            while conf < conf_thres: 
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

                _, _, conf=self.detections[i]



        elif mode == 'zigzag':
            states = ['left', 'forward', 'right', 'forward']
            current_state = states[0]
            forward_limit = 2.0 
            sideway_limit = 4.0 # move 4m left/right each time
            base_pos_x = self.x0 + sideway_limit / 2.0 * math.cos(original_yaw)
            base_pox_y = self.y0 - sideway_limit / 2.0 * math.cos(original_yaw)

            _, _, conf=self.detections[i]
            # Start looking
            while conf < conf_thres: 
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
                        rospy.loginfo('Switch from {} to {}'.format(states[1], states[1]))

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
                _, _, conf=self.detections[i]


        else:
            rospy.loginfo('Invalid look_around() mode!')
            return
        
        rospy.loginfo('Finished searching!')        
        return


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

        # print(self.detections)


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
                    self.bucket_seen=True
                    cv2.rectangle(img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), color, 2)
                    self.del_x=w/2-(rect[0]+rect[2]/2)
                    self.del_y=h/2-(rect[1]+rect[3]/2)


            combined_mask=cv2.bitwise_or(combined_mask, mask)
            i+=1

        combined_mask=cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        
        self.down_img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, combined_mask]), "bgr8"))

    def release_ball(self):
        print("ball released")

    def blind_mode(self):
        #toggling for off and on blind mode
        #motor controller passes cmd_vel from mission planner immediately without localization
        msg=Twist()
        msg.angular.x=50
        msg.angular.y=50
        self.cmd_vel_pub.publish(msg)



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
        




##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        Mission(nodename="mission", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
