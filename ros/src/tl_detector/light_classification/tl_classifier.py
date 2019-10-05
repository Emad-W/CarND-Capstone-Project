#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLight
import cv2 as cv
import numpy as np
import os
from matplotlib import pyplot as plt

dim = (256, 256)
color_thres = 10

class TLClassifier(object):
    def __init__(self):
        img_path = os.path.join("model/{}".format('template.jpeg'))
        self.template = cv.imread(img_path,0)
        self.template = cv.resize(self.template, (10,30))
        self.w, self.h = self.template.shape[::-1]
        
        methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
            'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
        #self.meth = methods[1]
        self.meth = 'cv.TM_SQDIFF'
        #self.meth = 'cv.TM_CCOEFF_NORMED'
        #self.meth = 'cv.TM_CCOEFF'
        plt.show()
        #img2_path = os.path.join("model/{}".format('red-157013445554.jpeg'))
        #self.img2 = cv.imread(img2_path)

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        img_color = cv.resize(image, dim)
        result = None
        found = False
        state = 3
        img = cv.cvtColor(img_color, cv.COLOR_BGR2GRAY)
        method = eval(self.meth)
        #print(img.shape)
        #print(template.shape)
        # Apply template Matching
        res = cv.matchTemplate(img,self.template,method)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        #if method  == cv.TM_SQDIFF or method == cv.TM_SQDIFF_NORMED:
        top_left = min_loc
            #print(1)
        #else:
            #print(2)
        #print(min_val, max_val, min_loc, max_loc)  
        #top_left = max_loc
        bottom_right = (top_left[0] + (self.w+10), top_left[1] + (self.h+20))
        #print(top_left,bottom_right)
        cv.rectangle(img,top_left, bottom_right, 255, 2)
        #print(top_left,bottom_right)

        for i in range(len(res)):
            if min(res[i]) < 0.99:
                found = True

        crop_img = img_color[np.clip(top_left[1]-20, 0, img.shape[1]):np.clip(bottom_right[1]+10, 0, img.shape[1]), np.clip(top_left[0]-10, 0, img.shape[0]):np.clip(bottom_right[0]+10, 0, img.shape[0])]

	#plt.imshow(img,cmap = 'gray')
        #plt.draw()
        #plt.pause(0.000000001)
	
        img_hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)

        ## Gen lower mask (0-5) and upper mask (175-180) of RED
        mask_r_1 = cv.inRange(img_hsv, (0,50,20), (10,255,255))
        mask_r_2 = cv.inRange(img_hsv, (170,50,20), (180,255,255))

        ## Merge the mask and crop the regions
        mask_r = cv.bitwise_or(mask_r_1, mask_r_2 )
        mask_g = cv.inRange(img_hsv,(40, 100, 20), (80, 255, 255))
        mask_g = cv.bitwise_or(mask_g, mask_g)
        mask_y = cv.inRange(img_hsv,(30, 50, 20), (35, 255, 255))
        mask_y = cv.bitwise_or(mask_y, mask_y)

        ## working on full picture

        img_hsv_full = cv.cvtColor(img_color, cv.COLOR_BGR2HSV)
        mask_r_1_full = cv.inRange(img_hsv_full, (0,50,20), (10,255,255))
        mask_r_2_full = cv.inRange(img_hsv_full, (170,50,20), (180,255,255))

        ## Merge the mask and crop the regions
        mask_r_full = cv.bitwise_or(mask_r_1_full, mask_r_2_full )
        
        color_count_r = cv.countNonZero(mask_r)
        color_count_g = cv.countNonZero(mask_g)
        color_count_y = 0 #cv.countNonZero(mask_y) # Removed as yellow will be considered as green
        color_count_r_full = cv.countNonZero(mask_r_full)
        #print (color_count_g,color_count_r,color_count_r_full)

        if color_count_r > (color_count_g+color_count_y + color_thres)*1.5:
                result = "red"
                state = 0
        elif color_count_g > (color_count_r+color_count_y + color_thres)*1.5:
                result = "green"
                state = 2
        elif color_count_y > (color_count_r + color_count_g)*1.5 :
                result = "yellow"
                state = 1

        elif found == False and color_count_r_full > 150:
                result = "red2"
                state = 0
        else:
                result = "unkown"
                state = 3



        #rospy.logwarn("TrafficLight classification: {0}".format(result))



        return state






