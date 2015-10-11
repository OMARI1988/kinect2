# roscore
roscore

# publish the kinect2 raw topics
sudo bash
rosrun kinect2_bridge kinect2_bridge 

# reads input from kinect2 and filter depth ! worth changing to filter depth at example
rosrun kinect2_viewer kinect2_pc_publisher

# subscriber and filter
# edit to include segmentation http://wiki.ros.org/pcl/Tutorials
rosrun pcl_functions filter

# add the visualziser
rosrun kinect2_viewer kinect2_pc_viewer 

# color and distance based segmentation
# http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php
