#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('kinect2_viewer')
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64

global x_1,x_2,y_1,y_2,z_1,z_2
x_1 = -0.39
x_2 = 0.19
y_1 = 0.23
y_2 = 0.79
z_1 = 0.2
z_2 = 0.87

if __name__ == '__main__':

    def x1(x):
        global x_1,x_2
        x_1 = (float(x)-100.0)/100.0
        if x_1 < x_2:
            print 'x1 = ',x_1
            pub_x1.publish(x_1)
        else:
            x_1 = x_2 - .01
            print 'increase x2'
            cv2.setTrackbarPos('x1: -1.00 to 1.00','image',int(x_1*100+100))

    def x2(x):
        global x_1,x_2
        x_2 = (float(x)-100.0)/100.0
        if x_2 > x_1:
            print 'x2 = ',x_2
            pub_x2.publish(x_2)
        else:
            x_2 = x_1 + .01
            print 'decrease x1'
            cv2.setTrackbarPos('x2: -1.00 to 1.00','image',int(x_2*100+100))

    def y1(x):
        global y_1,y_2
        y_1 = (float(x)-100.0)/100.0
        if y_1 < y_2:
            print 'y1 = ',y_1
            pub_y1.publish(y_1)
        else:
            y_1 = y_2 - .01
            print 'increase y2'
            cv2.setTrackbarPos('y1: -1.00 to 1.00','image',int(y_1*100+100))

    def y2(x):
        global y_1,y_2
        y_2 = (float(x)-100.0)/100.0
        if y_2 > y_1:
            print 'y2 = ',y_2
            pub_y2.publish(y_2)
        else:
            y_2 = y_1 + .01
            print 'decrease y1'
            cv2.setTrackbarPos('y2: -1.00 to 1.00','image',int(y_2*100+100))

    def z1(x):
        global z_1,z_2
        z_1 = float(x)/100.0
        if z_1 < z_2:
            print 'z1 = ',z_1
            pub_z1.publish(z_1)
        else:
            z_1 = z_2 - .01
            print 'increase z2'
            cv2.setTrackbarPos('z1: -1.00 to 1.00','image',int(z_1*100))

    def z2(x):
        global z_1,z_2
        z_2 = float(x)/100.0
        if z_2 > z_1:
            print 'z2 = ',z_2
            pub_z2.publish(z_2)
        else:
            z_2 = z_1 + .01
            print 'decrease z1'
            cv2.setTrackbarPos('z2: -1.00 to 1.00','image',int(z_2*100+100))

    def theta(x):
        th = float(x)*np.pi/180.0
        print 'theta = ',th
        pub_theta.publish(th)

    def distance(x):
        th = float(x)/100
        print 'distance = ',th
        pub_dis.publish(th)

    def color(x):
        th = float(x)/100
        print 'color = ',th
        pub_col.publish(th)

    def region(x):
        th = float(x)/100
        print 'region = ',th
        pub_reg.publish(th)

    def cluster_size(x):
        th = float(x)
        print 'cluster_size = ',th
        pub_cls.publish(th)





    def pc(x):
        if x == 0:
            print 'Point cloud OFF'
        if x == 1:
            print 'Point cloud ON'
        pub_pc.publish(x)

    def cluster(x):
        if x == 0:
            print 'Clusters OFF'
        if x == 1:
            print 'Clusters ON'
        pub_clusters.publish(x)

    def table(x):
        if x == 0:
            print 'table OFF'
        if x == 1:
            print 'table ON'
        pub_table.publish(x)

    def nothing(x):
        pass

    rospy.init_node('calibration', anonymous=True)
    pub_x1 = rospy.Publisher('x1', Float64, queue_size=10)
    pub_x2 = rospy.Publisher('x2', Float64, queue_size=10)
    pub_y1 = rospy.Publisher('y1', Float64, queue_size=10)
    pub_y2 = rospy.Publisher('y2', Float64, queue_size=10)
    pub_z1 = rospy.Publisher('z1', Float64, queue_size=10)
    pub_z2 = rospy.Publisher('z2', Float64, queue_size=10)
    pub_pc = rospy.Publisher('pc', Float64, queue_size=10)
    pub_clusters = rospy.Publisher('clusters', Float64, queue_size=10)
    pub_table = rospy.Publisher('table', Float64, queue_size=10)
    pub_theta = rospy.Publisher('theta', Float64, queue_size=10)


    pub_dis = rospy.Publisher('distance', Float64, queue_size=10)
    pub_col = rospy.Publisher('color', Float64, queue_size=10)
    pub_reg = rospy.Publisher('region', Float64, queue_size=10)
    pub_cls = rospy.Publisher('cluster_size', Float64, queue_size=10)

    # Create a black image, a window
    img = np.zeros((3,512,3), np.uint8)
    cv2.namedWindow('image')



    # create trackbars for color change
    cv2.createTrackbar('x1: -1.00 to 1.00','image',0,200,x1)
    cv2.createTrackbar('x2: -1.00 to 1.00','image',0,200,x2)
    cv2.createTrackbar('y1: -1.00 to 1.00','image',0,200,y1)
    cv2.createTrackbar('y2: -1.00 to 1.00','image',0,200,y2)
    cv2.createTrackbar('z1: 0.00 to 2.00','image',0,200,z1)
    cv2.createTrackbar('z2: 0.00 to 2.00','image',0,200,z2)
    cv2.createTrackbar('theta','image',0,359,theta)
    cv2.createTrackbar('phi','image',0,255,nothing)
    cv2.createTrackbar('psi','image',0,255,nothing)
    cv2.createTrackbar('distance','image',1,1200,distance)
    cv2.createTrackbar('color','image',1,16500,color)
    cv2.createTrackbar('region','image',1,11200,region)
    cv2.createTrackbar('cluster_size','image',1,1200,cluster_size)

    cv2.setTrackbarPos('x1: -1.00 to 1.00','image',int(x_1*100+100))
    cv2.setTrackbarPos('x2: -1.00 to 1.00','image',int(x_2*100+100))
    cv2.setTrackbarPos('y1: -1.00 to 1.00','image',int(y_1*100+100))
    cv2.setTrackbarPos('y2: -1.00 to 1.00','image',int(y_2*100+100))
    cv2.setTrackbarPos('z1: 0.00 to 2.00','image',int(z_1*100))
    cv2.setTrackbarPos('z2: 0.00 to 2.00','image',int(z_2*100))
    cv2.setTrackbarPos('theta','image',290)

    # create switch for ON/OFF functionality
    cv2.createTrackbar('point cloud :','image',0,1,pc)
    cv2.createTrackbar('clusters    :','image',0,1,cluster)
    cv2.createTrackbar('table       :','image',0,1,table)

    while(1):
        cv2.imshow('image',img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # get current positions of four trackbars
        # r = cv2.getTrackbarPos('R','image')
        # g = cv2.getTrackbarPos('G','image')
        # b = cv2.getTrackbarPos('B','image')
        # s = cv2.getTrackbarPos(switch,'image')

        # if s == 0:
        #     img[:] = 0
        # else:
        #     img[:] = [b,g,r]

    cv2.destroyAllWindows()
