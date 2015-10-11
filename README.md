# roscore
roscore

# publish the kinect2 raw topics
rosrun kinect2_bridge kinect2_bridge 

# reads input from kinect2 and filter depth ! worth changing to filter depth at example
rosrun kinect2_viewer kinect2_pc_publisher

# subscriber and filter
# edit to include segmentation http://wiki.ros.org/pcl/Tutorials
rosrun pcl_functions filter

rosrun pcl_functions color_xyz_clustering

rosrun kinect2_viewer kinect2_object_viewer

rosrun kinect2_viewer calibration.py
