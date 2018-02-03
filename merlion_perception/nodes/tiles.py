import math
import random
import cv2
import numpy as np
import matplotlib.pyplot as plt
import itertools
from ctypes import *

###########################
####Optiflow parameters####
###########################

######################
####Detection class###
######################

#define a class to receive the characteristics of each object detection
class Tile():

    def __init__(self, rect, ind):
        #bounding box parameters, center_x, center_y, width, height
        bb=rect
        self.bb=bb
        self.alive=True
        self.centers=[]
        self.centers.append((bb[0]+bb[2]/2, bb[1]+bb[3]/2))
        self.ind=ind

    def one_step_update(self, r):
        #pick one with best iou & iou>thres, if so update bb
        #if none detected, nevermind
        self.alive=False
        for i in range(len(r)):
            new_bb=r[i]
            iou, ios=self.get_iou(new_bb)
            #find box with max iou
            # print(iou)
            if iou>0.4:
                self.bb=new_bb  
                self.alive=True
                self.centers.append((self.bb[0]+self.bb[2]/2, self.bb[1]+self.bb[3]/2))
                break

        # print(self.ind, self.alive)
        if len(r)>0:
            r.pop(i)
            
        return r


    def get_iou(self, new_bb):
        #compute iou between current bb and new_bb

        #find intersection box
        x_a=int(max(self.bb[0], new_bb[0]))
        x_b=int(min(self.bb[0]+self.bb[2], new_bb[0]+new_bb[2]))
        y_a=int(max(self.bb[1], new_bb[1]))
        y_b=int(min(self.bb[1]+self.bb[3], new_bb[1]+new_bb[3]))

        #compute intersection area
        inter_area=(x_b-x_a)*(y_b-y_a)
        
        if (x_b-x_a)<0 or (y_b-y_a)<0:
            return 0, 0

        #compute each box's area
        x_min=int(self.bb[0])
        x_max=int(self.bb[0]+self.bb[2])
        y_min=int(self.bb[1])
        y_max=int(self.bb[1]+self.bb[3])
        cur_box_area=(x_max-x_min+1)*(y_max-y_min+1)


        x_min=int(new_bb[0])
        x_max=int(new_bb[0]+new_bb[2])
        y_min=int(new_bb[1])
        y_max=int(new_bb[1]+new_bb[3])

        new_box_area=(x_max-x_min+1)*(y_max-y_min+1)

        #compute the area ratio
        iou=inter_area/float(max(cur_box_area+new_box_area-inter_area, 0.00001))

        #compute inter/small_box
        small_area=min(self.bb[2]*self.bb[3], new_bb[2]*new_bb[3])
        if small_area<1:
            small_area=1
        #intersection over smaller
        ios=inter_area/float(small_area)

        return iou, ios



##########################
##########main############
##########################

