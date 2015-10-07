#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('kinect2_viewer')
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
import os

from os import listdir
from os.path import isfile, join
mypath = '/home/omari/kinect_cal_data/'
newpath = '/home/omari/Desktop/sync/'
onlyfiles = [ f for f in listdir(mypath) if isfile(join(mypath,f)) ]
for f in onlyfiles:
    if 'sync' in f:
        os.rename(mypath+f, newpath+f)
