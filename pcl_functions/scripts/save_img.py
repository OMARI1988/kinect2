#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs.msg
import visualization_msgs.msg
from cv_bridge import CvBridge
import cv2

class camera():
    """docstring for camera"""
    def __init__(self):
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.img = np.zeros((300,512,3), np.uint8)
        rospy.Subscriber('/kinect2_1/qhd/image_color', sensor_msgs.msg.Image, self._xtion_rgb)

    def _xtion_rgb(self,imgmsg):
        self.img = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

if __name__ == '__main__':

    file1 = 34

    rospy.init_node('calibration', anonymous=True)
    C = camera()

    while(1):
        cv2.imshow('image',C.img[130:,:820,:])
        k = cv2.waitKey(100) & 0xFF
        if k == 115:
            print 'saving : '+str(file1)
            if file1<10:
                cv2.imwrite('/home/omari/Datasets/Scenes/scene_000'+str(file1)+'.png',C.img[130:,:820,:])
            elif file1<100:
                cv2.imwrite('/home/omari/Datasets/Scenes/scene_00'+str(file1)+'.png',C.img[130:,:820,:])
            elif file1<1000:
                cv2.imwrite('/home/omari/Datasets/Scenes/scene_0'+str(file1)+'.png',C.img[130:,:820,:])
            file1 += 1
        if k == 27:
            break

    cv2.destroyAllWindows()
