#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <sys/timeb.h>
#include "baxter_core_msgs/EndpointState.h"
#include "baxter_core_msgs/EndEffectorState.h"


// void leftCallback (const baxter_core_msgs::EndpointState& msg);


std::string frame_str;
std::stringstream convert; // stringstream used for the conversion

std::stringstream folder;
std::string N("6");
int frame = 0;
bool flag1 = 0;
bool flag2 = 0;
bool flag3 = 0;

std::ofstream leftStream;
std::ofstream rightStream;

cv_bridge::CvImagePtr right;
cv_bridge::CvImagePtr kinect;
cv_bridge::CvImagePtr left;
baxter_core_msgs::EndpointState left_arm;
baxter_core_msgs::EndpointState right_arm;
baxter_core_msgs::EndEffectorState left_g;
baxter_core_msgs::EndEffectorState right_g;



void left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::imshow("left", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(1);
  if (!flag1)
    std::cout << "left recieved..." << std::endl;
  flag1 = 1;
  left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::imshow("right", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(1);
  if (!flag2)
    std::cout << "right recieved..." << std::endl;
  flag2 = 1;
  right = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void kinect_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::imshow("kinect", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(1);
  if (!flag3)
    std::cout << "kinect recieved..." << std::endl;
  flag3 = 1;
  kinect = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (flag1 && flag2 && flag3)
  {
  frame += 1;
  std::cout << frame << " received..." << std::endl;
  convert.str("");
  if (frame<10)
    convert << "000" << frame;
  else if (frame<100)
    convert << "00" << frame;
  else if (frame<1000)
    convert << "0" << frame;
  else if (frame<10000)
    convert << frame;
  frame_str = convert.str();
  cv::imwrite("/home/omari/Datasets/test"+std::string(N)+std::string("/Left_")+frame_str+std::string(".png"), left->image);
  cv::imwrite("/home/omari/Datasets/test"+std::string(N)+std::string("/Right_")+frame_str+std::string(".png"), right->image);
  cv::imwrite("/home/omari/Datasets/test"+std::string(N)+std::string("/Kinect_")+frame_str+std::string(".png"), kinect->image);
  std::string tmp_L = "/home/omari/Datasets/test" + std::string(N) + std::string("/EndpointLeft_") + frame_str + std::string(".txt");
  std::string tmp_R = "/home/omari/Datasets/test" + std::string(N) + std::string("/EndpointRight_") + frame_str + std::string(".txt");
	leftStream.open(tmp_L.c_str());
	rightStream.open(tmp_R.c_str());
  leftStream << "x:" << left_arm.pose.position.x << "\ny:" << left_arm.pose.position.y << "\nz:" << left_arm.pose.position.z << "\nrot_x:"
   << left_arm.pose.orientation.x << "\nrot_y:" << left_arm.pose.orientation.y << "\nrot_z:" << left_arm.pose.orientation.z << "\nrot_w:"
   << left_arm.pose.orientation.w << "\ngripper:" << left_g.position << std::endl;
  rightStream << "x:" << right_arm.pose.position.x << "\ny:" << right_arm.pose.position.y << "\nz:" << right_arm.pose.position.z << "\nrot_x:"
   << right_arm.pose.orientation.x << "\nrot_y:" << right_arm.pose.orientation.y << "\nrot_z:" << right_arm.pose.orientation.z << "\nrot_w:"
   << right_arm.pose.orientation.w << "\ngripper:" << right_g.position << std::endl;
	leftStream.close();
	rightStream.close();

  // Convert to PCL data type
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered2);
  folder.str("");
  folder << "/home/omari/Datasets/test" << std::string(N) << std::string("/tabletop_") << frame_str << std::string(".pcd");
  // std::cout << "saving " << "/home/omari/Datasets/test" + std::string(N) + std::string("/tabletop_") + frame_str + std::string(".pcd") << std::endl;
  pcl::io::savePCDFile (folder.str(), *cloud_filtered2, true);
  }
}


void leftCallback (const baxter_core_msgs::EndpointState& msg)
{
    left_arm = msg;
}

void rightCallback (const baxter_core_msgs::EndpointState& msg)
{
    right_arm = msg;
}

void rightGripperCallback (const baxter_core_msgs::EndEffectorState& msg)
{
    // std::cout << msg.position << std::endl;
    right_g = msg;
}

void leftGripperCallback (const baxter_core_msgs::EndEffectorState& msg)
{
    // std::cout << msg.position << std::endl;
    left_g = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("left");
  cv::namedWindow("right");
  cv::namedWindow("kinect");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("/cameras/left_hand_camera/image", 1, left_imageCallback);
  image_transport::Subscriber sub2 = it.subscribe("/cameras/right_hand_camera/image", 1, right_imageCallback);
  image_transport::Subscriber sub3 = it.subscribe("/kinect2/qhd/image_color", 1, kinect_imageCallback);
  ros::Subscriber sub = nh.subscribe ("/filtered_pointcloud", 1, cloud_cb);
	ros::Subscriber sub4 = nh.subscribe("/robot/limb/left/endpoint_state", 1, leftCallback);
	ros::Subscriber sub5 = nh.subscribe("/robot/limb/right/endpoint_state", 1, rightCallback);
	ros::Subscriber sub6 = nh.subscribe("/robot/end_effector/right_gripper/state", 1, rightGripperCallback);
	ros::Subscriber sub7 = nh.subscribe("/robot/end_effector/left_gripper/state", 1, leftGripperCallback);
  ros::spin();
  cv::destroyWindow("left");
  cv::destroyWindow("right");
  cv::destroyWindow("kinect");
}
