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
    # sleeping time
    timestep = 0.1

    #cmd_vel speeds, in m/s and rad/s
    forward_speed = 4*timestep
    side_speed = 2*timestep
    yaw_speed = 10*math.pi/180*timestep

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
    seq = [0]

    #stores detections, row wise: gate, bucket, flare, col wise: x, y, confidence
    detections = np.zeros((3, 3))

    #visual servoing params
    blue_del_x, blue_del_y, streak = 0, 0, 0
    bucket_seen = False

    #look around bias, estimated global position of gate, bucket, and flare
    detection_bias = [[9, -3], [25, -4], [19, -6]]
    # detection_bias=[[0, 0], [0, 0], [0, 0]]

    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()

        ####Subscribers####
        #sub to heatmaps from detector
        rospy.Subscriber("/detection/heatmap", Image, self.heatmap_callback, queue_size = 1)
        
        #sub to downward cam as main for bucket
        rospy.Subscriber("/down/image_rect_color", Image, self.down_img_callback, queue_size = 1)
        # rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.down_img_callback, queue_size = 1)

        #sub odom
        rospy.Subscriber('/visual_odom', Odometry, self.odom_callback, queue_size=1)
        while not self.odom_received and not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo("Waiting for odom...")

        ####Publishers####
        self.cmd_vel_pub = rospy.Publisher('/merlion/control/cmd_vel', Twist, queue_size=1)
        self.front_img_pub = rospy.Publisher('/mission/front_img', Image, queue_size=1)
        self.down_img_pub = rospy.Publisher('/mission/down_img', Image, queue_size=1)

        for i in self.seq:
            if i == 0:
                self.mission_0(distance=1)
            elif i == 1:
                self.mission_1()
            elif i == 2:
                self.mission_2()
            elif i == 3:
                self.mission_3()

    def mission_0(self, distance=10):
        ####qualification####
        #1.move 10m forward
        #2.move 10m reverse.
        #3.while holding yaw=0 and depth

        rospy.loginfo("init qualification task")
        forward_done=False
        

        while not rospy.is_shutdown():
            if self.x0 < distance + 1 and forward_done == False:
                rospy.loginfo("0.1 moving forward")
                self.pub_cmd_vel(self.forward_speed, 0, 0)
            else:
                rospy.loginfo("0.2 reversing")
                forward_done=True
                self.pub_cmd_vel(-self.forward_speed, 0, 0)
                if self.x0<1:
                    break

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
        self.look_around(0, 'yawing', 15)

        #distance threshold in sideway movement
        thres=0.2
        #move this much pass the gate
        offset=2
        #step 4 enclosing step 2 and step 3
        while not rospy.is_shutdown():
            x, y, conf=self.detections[0]
            print(x, y)
            #step 2
            error_y=y-self.y0
            if abs(error_y)>thres:
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
        #9. rotate 
        ###########################
        rospy.loginfo("init mission 2")
        
        thres=0.2
        offset=-2
        #step 1

        # self.look_around(1, 'yawing', 10)
        
        # #step 4 enclosing 2 and 3
        # while not rospy.is_shutdown():
        #     x, y, conf=self.detections[1]

        #     #step 2
        #     error_y=y-self.y0
        #     if abs(error_y)>thres:
        #         rospy.loginfo("2.2 moving sideway")
        #         sign=np.sign(error_y)
        #         self.pub_cmd_vel(0, sign*self.side_speed,0)
        #     else:
        #         #step 3
                
        #         if x+offset-self.x0>0:
        #             rospy.loginfo("2.3 move forward")
        #             self.pub_cmd_vel(self.forward_speed, 0, 0)
        #         else:
        #             #bravo we're near the bucket
        #             break 

        # #step 5
        # #tell motor controller to switch to blind mode
        # self.blind_mode()
        # #set few timesteps to foward amount of 2 m ##TODO tune ts
        # ts=1
        # for i in range(int(ts/self.timestep)):
        #     rospy.loginfo("blind motion")
        #     self.pub_cmd_vel(self.forward_speed, 0, 0)
        #     if rospy.is_shutdown():
        #         return
            
        #step 6
        sign=0
        while not rospy.is_shutdown():   
            
            k=self.forward_speed/320

            if self.bucket_seen is True:
                rospy.loginfo("2.6 visual servo adjusting to bucket")
                #body x axis is in image +y direction
                #body y axis is in image +x direction
                vs_x=self.blue_del_y*k
                vs_y=self.blue_del_x*k
                self.pub_cmd_vel(vs_x, vs_y, 0)
                print(vs_x, vs_y, streak)

                if abs(vs_x)<0.2 and abs(vs_y)<0.2 and self.streak>5:
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
        self.release_ball()
        rospy.sleep(3)

        #step 8
        #set few timesteps to foward
        ts=3
        for i in range(int(ts/self.timestep)):
            rospy.loginfo("blind motion")
            self.pub_cmd_vel(-self.forward_speed, 0, 0)
            if rospy.is_shutdown():
                return
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

        rospy.loginfo("mission 2 success")

    def mission_3(self):
        ####hit da flare!!!####
        #1. look around until confident
        #2. yaw to facing flare directly
        #3. move forward 2m
        #4. redo 2 and 3 until passes flare
        ######################
        rospy.loginfo("init mission 3")

        #step 1
        self.look_around(2, 'zigzag', 15)
        
        #step 4 enclosing 2 and 3
        while not rospy.is_shutdown():            
            x, y, conf=self.detections[2]

            #step 2
            yaw_des=math.atan2(y-self.y0, x-self.x0)
            error_yaw=yaw_des-self.yaw0
            error_yaw=math.atan2(math.sin(error_yaw), math.cos(error_yaw))
            ang_thres=1*math.pi/180

            rospy.loginfo(error_yaw*180/math.pi)
            if abs(error_yaw)>ang_thres:# and error_dis>dis_thres:
                rospy.loginfo("3.2 yawing facing flare")
                sign=np.sign(error_yaw)
                

                self.pub_cmd_vel(0, 0, sign*self.yaw_speed)

            else:
                #step 3
                rospy.loginfo("3.3 move forward")
                self.pub_cmd_vel(self.forward_speed, 0, 0)
                if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<0.5:
                    break
        #step 
        #set few timesteps to foward
        ts=3
        for i in range(int(ts/self.timestep)):
            self.pub_cmd_vel(self.forward_speed, 0, 0)

        rospy.loginfo("mission 3 success")

    def angle_diff(self, minuend, subtrahend): 
        diff = minuend - subtrahend
        return math.atan2(math.sin(diff), math.cos(diff))

    def look_around(self, i, mode, conf_thres=20):
        txt=str(i+1)+".1 lookaround"
        rospy.loginfo(txt)

        bias=self.detection_bias[i]
        rospy.loginfo(bias)


        if bias[0]!=0 or bias[1]!=0:
            r=10
            #go to proximity of bias
            yaw_des=math.atan2(bias[1]-self.y0, bias[0]-self.x0)
            x=bias[0]
            y=bias[1]

            while not rospy.is_shutdown():            

                error_yaw=yaw_des-self.yaw0
                error_yaw=math.atan2(math.sin(error_yaw), math.cos(error_yaw))
                ang_thres=1*math.pi/180
                if abs(error_yaw)>ang_thres:# and error_dis>dis_thres:
                    # rospy.loginfo("yawing towards bias")
                    sign=np.sign(error_yaw)
                    self.pub_cmd_vel(0, 0, sign*self.yaw_speed)
                else:
                    #step 3
                    # rospy.loginfo("forward towards bias")
                    self.pub_cmd_vel(self.forward_speed, 0, 0)

                    if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<r:
                        break

        if mode == 'yawing':
            states = ['ccw_yaw', 'cw_yaw', 'return_yaw', 'forward']
            current_state = states[0]
            base_pos_x = self.x0
            base_pos_y = self.y0
            yaw_limit = 45.0  * math.pi/ 180.0
            trl_limit = 1.5 # forward 2m each time
            ang_thres=0.03

            if bias[0]==0 and bias[1]==0:
                #no bias
                original_yaw = self.yaw0
            else:
                original_yaw= math.atan2(bias[1]-self.y0, bias[0]-self.x0)

            x, y, conf=self.detections[i]
            # Start looking
            while conf < conf_thres and not rospy.is_shutdown(): 
                if current_state == states[0]:
                    self.pub_cmd_vel(0, 0, self.yaw_speed)
                    if self.angle_diff(self.yaw0, original_yaw) >= yaw_limit: # check to switch state
                        current_state = states[1]
                        rospy.loginfo('Switch from {} to {}'.format(states[0], states[1]))
                
                elif current_state == states[1]:
                    self.pub_cmd_vel(0, 0, -self.yaw_speed)
                    if self.angle_diff(self.yaw0, original_yaw) <= -yaw_limit:
                        current_state = states[2]
                        rospy.loginfo('Switch from {} to {}'.format(states[1], states[2]))

                elif current_state == states[2]:
                    self.pub_cmd_vel(0, 0, self.yaw_speed)
                    if abs(self.angle_diff(self.yaw0, original_yaw)) < ang_thres: # threshold to go back to original yaw, maybe need something more accurate for this
                        current_state = states[3]
                        rospy.loginfo('Switch from {} to {}'.format(states[2], states[3]))

                elif current_state == states[3]:
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if diff_x * diff_x + diff_y * diff_y > trl_limit * trl_limit:
                        current_state = states[0]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('Switch from {} to {}'.format(states[3], states[0]))

                x, y, conf=self.detections[i]

                    
                if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<2:
                    break



            #face forwward
            while not rospy.is_shutdown():
                error_yaw=self.angle_diff(0, self.yaw0)
                if abs(error_yaw) < ang_thres: # threshold to go back to original yaw, maybe need something more accurate for this
                    break
                else:
                    sign=np.sign(error_yaw)
                    self.pub_cmd_vel(0, 0, sign*self.yaw_speed)

        elif mode == 'zigzag':
            states = ['left', 'forward_l', 'right', 'forward_r']
            current_state = states[0]
            forward_limit = 1.5 
            sideway_limit = 4.0 # move 4m left/right each time
            original_yaw = self.yaw0

            if bias[0]==0 and bias[1]==1:
                base_pos_x = self.x0 + sideway_limit / 2.0 * math.sin(original_yaw)
                base_pos_y = self.y0 - sideway_limit / 2.0 * math.cos(original_yaw)
            else:
                del_x=bias[0]-self.x0
                del_y=bias[1]-self.y0
                diff_y=del_y*math.cos(self.yaw0)-del_x*math.sin(self.yaw0)
                print(diff_y)
                base_pos_x = self.x0 + sideway_limit / 2.0 * math.sin(original_yaw)
                base_pos_y = self.y0 + diff_y - sideway_limit / 2.0 * math.cos(original_yaw)
            x, y, conf=self.detections[i]

            # Start looking
            while conf < conf_thres and not rospy.is_shutdown(): 

                if current_state == states[0]:
                    self.pub_cmd_vel(0, self.side_speed, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if math.sqrt(diff_x**2 + diff_y**2) > sideway_limit:
                        current_state = states[1]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('0. Switch from {} to {}'.format(states[0], states[1]))
                
                elif current_state == states[1]:

                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if math.sqrt(diff_x**2 + diff_y**2) > forward_limit:
                        current_state = states[2]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('1. Switch from {} to {}'.format(states[1], states[2]))

                elif current_state == states[2]:
                    self.pub_cmd_vel(0, -self.side_speed, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if math.sqrt(diff_x**2 + diff_y**2)> sideway_limit:
                        current_state = states[3]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('2. Switch from {} to {}'.format(states[2], states[3]))
                
                elif current_state == states[3]:
                    self.pub_cmd_vel(self.forward_speed, 0, 0)
                    diff_x = self.x0 - base_pos_x
                    diff_y = self.y0 - base_pos_y
                    if math.sqrt(diff_x**2 + diff_y**2) > forward_limit:

                        current_state = states[0]
                        base_pos_x = self.x0
                        base_pos_y = self.y0
                        rospy.loginfo('3. Switch from {} to {}'.format(states[3], states[0]))

                x, y, conf=self.detections[i]
                if math.sqrt((x-self.x0)**2+(y-self.y0)**2)<2:
                    break

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

        # rospy.loginfo(self.detections)


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
                    if i==0:
                        print("red bucket")
                    elif i==1:

                        print("blue bucket")
                        self.blue_del_x=del_x
                        self.blue_del_y=del_y

                        #if sub inside bucket
                        if rect[0]<w/2 and rect[2]+rect[0]>w/2 and rect[1]<h/2 and rect[1]+rect[3]>h/2:
                            self.streak+=1
                        else:
                            self.streak=0
                    print(del_x, del_y)
                    # rospy.loginfo(self.bucket_seen, self.del_x, self.del_y)

            combined_mask=cv2.bitwise_or(combined_mask, mask)
            i+=1

        combined_mask=cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        
        self.down_img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, combined_mask]), "bgr8"))

    def release_ball(self):
        rospy.loginfo("ball released")

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
        rospy.sleep(self.timestep)

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
        # rospy.loginfo(self.z0)
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
