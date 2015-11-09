#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('kinect2_viewer')
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
import os
import time
import sensor_msgs.msg



if __name__ == '__main__':

        folder = len([x[0] for x in os.walk("/home/omari/Datasets/Static_Scenes/")])
        for x in range(1,folder):
            if x<10:
                dir1 = "/home/omari/Datasets/Static_Scenes/scene_000"+str(x)+"/scene_image.png"
            elif x<100:
                dir1 = "/home/omari/Datasets/Static_Scenes/scene_00"+str(x)+"/scene_image.png"
            elif x<1000:
                dir1 = "/home/omari/Datasets/Static_Scenes/scene_0"+str(x)+"/scene_image.png"
            print dir1
            img = cv2.imread(dir1)
            cv2.imshow('xtion rgb',img)
            k = cv2.waitKey(100) & 0xff
