#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('kinect2_viewer')
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from scipy import linalg
from sklearn import mixture

#listener
def listen():
    rospy.init_node('rgb_clustering', anonymous=True)
    rospy.Subscriber("/filtered_pointcloud", PointCloud2, callback_kinect)

def callback_kinect(data) :
    ################################################ building the data points
    print 'building the datapoints...'
    counter = 0
    X = np.zeros((data.width, 3))
    for i in range(data.height):
        for j in range(data.width):
            data_out =  pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[j, i]])
            int_data = next(data_out)
            if int_data[2]>=0 and int_data[2]<=1 and int_data[0]>=-1 and int_data[0]<=1 and int_data[1]>=-1 and int_data[1]<=1:
                if int_data[0:3] != (0,0,0,) and int_data[0:3] != (0,0,1,):
                    A = np.array(list(int_data)[0:3])
                    # if X == []:         X = [A]
                    X[counter,:] = A
                    counter+=1
            if counter == 10000:
                break
    X = X[0:counter,:]
    # print X

    ################################################ clustering
    print 'clustering...'
    lowest_bic = np.infty
    bic = []
    n_components_range = range(1, 9)
    cv_types = ['full']
    for cv_type in cv_types:
        for n_components in n_components_range:
            # Fit a mixture of Gaussians with EM
            gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type)
            gmm.fit(X)
            bic.append(gmm.bic(X))
            if bic[-1] < lowest_bic:
                lowest_bic = bic[-1]
                best_gmm = gmm
    print best_gmm

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
