#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('kinect2_viewer')
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, PointCloud

#listener
def listen():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/point1", PointCloud2, callback_kinect)

def callback_kinect(data) :
    # print data.height
    # print data.width
    # print data.point_step
    # print data.row_step
    # pick a height
    height =  int (data.height / 2)
    # pick x coords near front and center
    middle_x = int (data.width / 2)
    # examine point
    middle = read_depth (middle_x, height, data)
    # print 'it worked'
    # do stuff with middle
    print middle


def read_depth(width, height, data) :
    # read function
    if (height >= data.height) or (width >= data.width) :
        return -1
    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height],[width+1, height+1]])
    # print data_out
    int_data = next(data_out)
    # rospy.loginfo("int_data " + str(int_data))
    return int_data


if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
