#!/usr/bin/env python
#Reinaldo Maslim, NTU Merlion 2018

import rospy
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseArray, Vector3
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker
import tf
import time

import random

from tiles import Tile

#################################
##############class##############
#################################

class Localizer(object):
    skip=2
    
    frame_counter=0
    ind_count=0

    pos_x=0
    pos_y=0

    last_yaw=0
    last_height=2.6

    last_yaw_count=0

    tiles=[]
    colors=[]

    grad_pred=[]
    height_pred=[]

    imu_roll=0
    imu_pitch=0
    # imu_yaw=0
    imu_yaw=0

    first=True

    #stores absolute imu direction of pool frame, if not avail set to 0
    yaw_init_offset=125*math.pi/180

    with_visual_correction=True


    def __init__(self, nodename, drive=None):
        rospy.init_node(nodename, anonymous=False)
        self.bridge = CvBridge()
        self.init_colors()

        ####Subscribers####
        #sub to downward cam as main for localizer
        rospy.Subscriber("/down/image_rect_color", Image, self.img_callback, queue_size = 1)
        # rospy.Subscriber("/logi_c310/usb_cam_node/image_raw", Image, self.img_callback, queue_size = 1)
        
        #sub to imu
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback, queue_size=1)

        ####Publishers####
        self.img_pub=rospy.Publisher('/localizer/img', Image, queue_size=1)
        self.vodom_pub=rospy.Publisher('/visual_odom', Odometry, queue_size=1)

        
        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    def cor_imu_callback(self, msg):
        self.imu_roll, self.imu_pitch, self.imu_yaw=msg.x, msg.y, msg.z

    def imu_callback(self, msg):
        # msg.orientation

        self.imu_roll, self.imu_pitch,  yaw= euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))        
        print(yaw*180/math.pi)
        self.imu_yaw=yaw-self.yaw_init_offset
        
        if self.first and self.yaw_init_offset==0:
            self.yaw_init_offset=self.imu_yaw
            
        if self.first:
            self.first=False
        # print(self.imu_yaw)

    def img_callback(self, msg):
        start_time=time.time()
        # print(len(self.tiles))
        font = cv2.FONT_HERSHEY_SIMPLEX
        color=(0, 0, 255)

        if self.frame_counter%self.skip==0:
            # self.tiles=[]
            img=self.bridge.imgmsg_to_cv2(msg, "bgr8")

            res=img.copy()
            h, w = img.shape[:2]

            M = cv2.getRotationMatrix2D((w/2,h/2),(self.imu_yaw)*180/math.pi,1)
            img = cv2.warpAffine(img,M,(w,h))

            #circle mask
            circle_mask=np.zeros_like(img)
            circle_mask=cv2.circle(circle_mask, (w/2, h/2), h/2, [255, 255, 255], -1)
            circle_mask=circle_mask[:, :, 0]

            # print(h,w)
            img=self.img_correction(img)
            
            blur = cv2.GaussianBlur(img,(7, 7),0)
            hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

            mask = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                        cv2.THRESH_BINARY,21, 2)
            
            kernel = np.ones((5,5),np.uint8)    
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            opening=255-opening
            opening = cv2.dilate(opening, None, iterations=1)
            contour_mask=255-opening
            opening[circle_mask==0]=0
            #fit lines to extract major direction
            minLineLength=100
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
                # cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                cv2.line(contour_mask, (x1, y1), (x2, y2), 0, 1, cv2.LINE_AA)

            hist, bin_edges = np.histogram(grad, density=False)
            ind=np.argmax(hist)
            best_grad=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)
            
            ind=np.where(np.abs(grad-best_grad)<10)
            good_grads=grad[ind]
            best_grad=np.mean(good_grads)

            
            # contour_mask=self.mask_correction(contour_mask)
            M = cv2.getRotationMatrix2D((w/2,h/2),best_grad,1)
            contour_mask = cv2.warpAffine(contour_mask,M,(w,h))

            (_,contours,_) = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contour_mask=cv2.cvtColor(contour_mask, cv2.COLOR_GRAY2BGR)            
            areas=[]
            border=0
            r=[]

            for contour in contours:
                rect = cv2.boundingRect(contour)

                if rect[0]>border and rect[0]+rect[2]<w-border and rect[1]>border and rect[3]+rect[1]<h-border:
                    area=int(rect[3]*rect[2])
                    # print(area)
                    ar=float(rect[2])/rect[3]
                    real_ar=0.25/0.12
                    if area>1000 and area<120000 and abs(ar/real_ar-1)<0.3:
                        cv2.rectangle(contour_mask, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), (0,255,0), 2)
                        areas.append(area)
                        r.append(rect)

            areas=np.asarray(areas)
            hist, bin_edges = np.histogram(areas, bins='fd', density=False)
            ind=np.argmax(hist)
            # best_area=(bin_edges[ind]+bin_edges[ind+1])/2

            best_area=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)
            ind=np.where(np.abs(areas-best_area)<0.1*best_area)
            if len(ind)>5:
                good_areas=areas[ind]
                best_area=np.mean(good_areas)


            pred_depth=self.predict_depth(best_area)

            pred_depth=pred_depth*math.cos(self.imu_pitch)*math.cos(self.imu_roll)

            for tile in self.tiles:
                r=tile.one_step_update(r)

            for rect in r:
                self.tiles.append(Tile(rect, self.ind_count))
                self.ind_count+=1

            for tile in self.tiles:
                if tile.alive == False:
                    self.tiles.remove(tile)

            del_x=[]
            del_y=[]
            for tile in self.tiles:
                color=self.colors[tile.ind%20]
                if len(tile.centers)>2:
                    del_x.append(tile.centers[-1][1]-tile.centers[-2][1])
                    del_y.append(tile.centers[-1][0]-tile.centers[-2][0])
                contour_mask = cv2.circle(contour_mask, (int(tile.centers[-1][0]), int(tile.centers[-1][1])), 5, color, -1)
                cv2.putText(contour_mask, str(tile.ind), (tile.bb[0]+10, tile.bb[1]+10), font, 0.8, color, 1, cv2.LINE_AA)

            hist, bin_edges = np.histogram(np.asarray(del_x), bins='fd', density=False)
            ind=np.argmax(hist)
            best_del_x=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)

            hist, bin_edges = np.histogram(np.asarray(del_y), bins='fd', density=False)
            ind=np.argmax(hist)
            best_del_y=round((bin_edges[ind]+bin_edges[ind+1])/2, 2)


            #tile real world dimension
            fov_w, fov_h=48*math.pi/180, 36*math.pi/180
            px_W, px_H=640, 480        
            W=2*pred_depth*math.tan(fov_w/2)+0.0001
            ppm=px_W/W
            self.pos_x-=best_del_x/ppm
            self.pos_y-=best_del_y/ppm
            self.pub_odom(self.pos_x, self.pos_y, pred_depth, best_grad)

            # print(best_grad, best_area, pred_depth)
            cv2.rectangle(contour_mask, (0, 0), (w, 80), (0,0,0), -1)

            text="direction "+str(round(best_grad+self.imu_yaw*180/math.pi, 2))+", height: "+str(round(pred_depth,2)) +"m"
            text2="x: " + str(round(self.pos_x, 2))+"m, y: " +str(round(self.pos_y, 2))+"m"
            color=(255,255,255)
            cv2.putText(contour_mask, text, (50, 20), font, 0.8, color, 1, cv2.LINE_AA)
            cv2.putText(contour_mask, text2, (50, 60), font, 0.8, color, 1, cv2.LINE_AA)


            opening=cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(np.hstack([img, contour_mask]), "bgr8"))

        self.frame_counter+=1

        print("total time: "+str(time.time()-start_time))


    def predict_depth(self, area):
        if area<10:
            return 0
        #fov of camera
        fov_w, fov_h=48*math.pi/180, 36*math.pi/180
        px_W, px_H=640, 480

        #tile real world dimension
        real_w, real_h=0.25, 0.12

        #pixel tile size
        px_w=math.sqrt(area/(real_w*real_h))*real_w
        px_h=math.sqrt(area/(real_w*real_h))*real_h
        # print(px_w, px_h)
        
        #camera fov in meters
        W=px_W*real_w/px_w
        H=px_H*real_h/px_h
        # print(W, H)
        #predict depth
        d_w=W/(2*math.tan(fov_w/2))
        d_h=H/(2*math.tan(fov_h/2))
        # print(d_w, d_h)
        return (d_w+d_h)/2


    def img_correction(self, img):
        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(5, 5))
        res=np.zeros_like(img)
        for i in range(3):
            res[:, :, i] = clahe.apply(img[:, :, i])
        return res

    def mask_correction(self, mask):
        res=np.zeros_like(mask)
        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(10, 10))
        res = clahe.apply(mask)
        return res

    def rand_color(self):
        a = random.randrange(0,256)
        b = random.randrange(0,256)
        c = random.randrange(0,256)

        return np.array([a, b, c])

    def init_colors(self):

        for i in range(20):
            self.colors.append(self.rand_color())

    def pub_odom(self, x, y, h, yaw):
        print("-----")
        v_yaw=yaw*math.pi/180
        print(yaw, self.imu_yaw*180/math.pi)
        if self.with_visual_correction:
            yaw=yaw*math.pi/180+self.imu_yaw
            if self.first:
                thres=math.pi
            else:
                thres=math.pi/30

            if abs(v_yaw)<thres:
                self.yaw_init_offset-=v_yaw
                self.yaw_init_offset=math.atan2(math.sin(self.yaw_init_offset), math.cos(self.yaw_init_offset))
        else:
            yaw=self.imu_yaw

        yaw=math.atan2(math.sin(yaw), math.cos(yaw))

        if not self.first and abs(self.last_yaw-yaw)>20*math.pi/180 and self.last_yaw_count<3:
            yaw=self.last_yaw
            self.last_yaw_count+=1
        elif h==0:
            yaw=self.last_yaw
            self.last_yaw_count+=1
        else:
            self.last_yaw_count=0


        self.grad_pred.append(yaw*180/math.pi)
        if len(self.grad_pred)>20:
            self.grad_pred.pop(0)

        fil_grad=self.g_h_filter(self.grad_pred)
        # print(yaw*180/math.pi, fil_grad)
        yaw=fil_grad*math.pi/180

        thres=0.9
        # print(yaw*180/math.pi)
        # if not self.first and h-self.last_height>thres:
        #     h=self.last_height
        # elif h==0:
        #     h=self.last_height-0.1
        #     h=np.clip(h, 0, 1)
        # elif h<0.2:
        #     h=self.last_height


        self.height_pred.append(h)
        if len(self.height_pred)>20:
            self.height_pred.pop(0)
        fil_h=self.g_h_filter(self.height_pred)
        h=fil_h

        self.last_height=h
        self.last_yaw=yaw

        #offset with cam to baselink
        h+=0.3

        #if it's the first time, memorize its initial readings
        br = tf.TransformBroadcaster()

        br.sendTransform((x, y, h),
                         tf.transformations.quaternion_from_euler(self.imu_roll, self.imu_pitch, yaw),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        
        #publish odometry
        odom=Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x=x
        odom.pose.pose.position.y=y
        odom.pose.pose.position.z=h
        q=Quaternion()
        q.x, q.y, q.z, q.w=tf.transformations.quaternion_from_euler(self.imu_roll, self.imu_pitch, yaw)
        odom.pose.pose.orientation=q
        self.vodom_pub.publish(odom)

        
        

    def g_h_filter(self, data, dx=1., g=6./10, h=2./3, dt=1.):
        x_est = data[0]

        for z in data:
            # prediction step
            x_pred = x_est + (dx*dt)
            dx = dx

            # update step
            residual = z - x_pred
            dx = dx + h * (residual) / dt
            x_est = x_pred + g * residual
        
        return x_est


##########################
##########main############
##########################



if __name__ == '__main__':

    try:
        Localizer(nodename="localizer", drive=None)
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")
