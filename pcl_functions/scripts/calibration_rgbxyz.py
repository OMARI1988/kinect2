#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64


if __name__ == '__main__':

    def x1f(x):
        pub_x1 = rospy.Publisher('distance', Float64, queue_size=10)
        x1 = cv2.getTrackbarPos('distance','image')/10.0
        pub_x1.publish(x1)

    def x2f(x):
        pub_x2 = rospy.Publisher('color', Float64, queue_size=10)
        x2 = cv2.getTrackbarPos('color','image')/10.0
        pub_x2.publish(x2)

    def y1f(x):
        pub_y1 = rospy.Publisher('region', Float64, queue_size=10)
        y1 = cv2.getTrackbarPos('region','image')/10.0
        pub_y1.publish(y1)

    def y2f(x):
        pub_y2 = rospy.Publisher('cluster_size', Float64, queue_size=10)
        y2 = cv2.getTrackbarPos('cluster_size','image')
        pub_y2.publish(y2)

    def z1f(x):
        pub_z1 = rospy.Publisher('z1', Float64, queue_size=10)
        z1 = cv2.getTrackbarPos('z1','image')/1000.0 - 1
        pub_z1.publish(z1)

    def z2f(x):
        pub_z2 = rospy.Publisher('z2', Float64, queue_size=10)
        z2 = cv2.getTrackbarPos('z2','image')/1000.0 - 1
        pub_z2.publish(z2)

    def nothing(x):
        pass

    rospy.init_node('calibration', anonymous=True)
    # Create a black image, a window
    img = np.zeros((300,512,3), np.uint8)
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('distance','image',0,10000,x1f)
    cv2.createTrackbar('color','image',0,20000,x2f)
    cv2.createTrackbar('region','image',0,20000,y1f)
    cv2.createTrackbar('cluster_size','image',0,20000,y2f)
    cv2.createTrackbar('z1','image',0,2000,z1f)
    cv2.createTrackbar('z2','image',0,2000,z2f)

    # create switch for ON/OFF functionality
    switch = '0 : OFF \n1 : ON'
    cv2.createTrackbar(switch, 'image',0,1,nothing)

    while(1):
        cv2.imshow('image',img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # get current positions of four trackbars
        # x1 = cv2.getTrackbarPos('x1','image')
        # x2 = cv2.getTrackbarPos('x2','image')
        # y1 = cv2.getTrackbarPos('y1','image')
        # y2 = cv2.getTrackbarPos('y2','image')
        # z1 = cv2.getTrackbarPos('z1','image')
        # z2 = cv2.getTrackbarPos('z2','image')

    cv2.destroyAllWindows()
