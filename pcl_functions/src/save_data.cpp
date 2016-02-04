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
#include <sys/stat.h>
#include <boost/filesystem.hpp>

// void leftCallback (const baxter_core_msgs::EndpointState& msg);


std::string frame_str;
std::stringstream convert; // stringstream used for the conversion

std::string folder1;
std::string sub_folder1;

std::stringstream folder;
std::stringstream N;
int frame = 0;
bool flag1 = 0;
bool flag2 = 0;
bool flag3 = 0;
bool flag4 = 0;

struct stat info;
std::ofstream leftStream;
std::ofstream rightStream;

cv_bridge::CvImagePtr right;
cv_bridge::CvImagePtr kinect;
cv_bridge::CvImagePtr left;
baxter_core_msgs::EndpointState left_arm;
baxter_core_msgs::EndpointState right_arm;
baxter_core_msgs::EndEffectorState left_g;
baxter_core_msgs::EndEffectorState right_g;
sensor_msgs::PointCloud2ConstPtr table;

pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);

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
cloud_table (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  table = cloud_msg;
  if (!flag4)
    std::cout << "table pointcloud recieved..." << std::endl;
  flag4 = 1;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (flag1 && flag2 && flag3 && flag4)
  {
  frame += 1;
  std::cout << frame << "objects received..." << std::endl;
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
  // saving left hand image
  sub_folder1 = folder1+"/LH_rgb/Left_"+frame_str+".png";
  cv::imwrite(sub_folder1.c_str(), left->image);
  // saving right hand image
  sub_folder1 = folder1+"/RH_rgb/Right_"+frame_str+".png";
  cv::imwrite(sub_folder1.c_str(), right->image);
  // saving kinect image
  sub_folder1 = folder1+"/kinect_rgb/Kinect_"+frame_str+".png";
  cv::imwrite(sub_folder1.c_str(), kinect->image);
  // saving left hand data
  sub_folder1 = folder1+"/LH_data/EndpointLeft_"+frame_str+".csv";
	leftStream.open(sub_folder1.c_str());
  leftStream << "x:" << left_arm.pose.position.x << "\ny:" << left_arm.pose.position.y << "\nz:" << left_arm.pose.position.z << "\nrot_x:"
   << left_arm.pose.orientation.x << "\nrot_y:" << left_arm.pose.orientation.y << "\nrot_z:" << left_arm.pose.orientation.z << "\nrot_w:"
   << left_arm.pose.orientation.w << "\ngripper:" << left_g.position << std::endl;
	leftStream.close();
  // saving right hand data
  sub_folder1 = folder1+"/RH_data/EndpointRight_"+frame_str+".csv";
	rightStream.open(sub_folder1.c_str());
  rightStream << "x:" << right_arm.pose.position.x << "\ny:" << right_arm.pose.position.y << "\nz:" << right_arm.pose.position.z << "\nrot_x:"
   << right_arm.pose.orientation.x << "\nrot_y:" << right_arm.pose.orientation.y << "\nrot_z:" << right_arm.pose.orientation.z << "\nrot_w:"
   << right_arm.pose.orientation.w << "\ngripper:" << right_g.position << std::endl;
	rightStream.close();
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered2);
  sub_folder1 = folder1+"/tabletop_pc/tabletop_pc_"+frame_str+".pcd";
  pcl::io::savePCDFile (sub_folder1.c_str(), *cloud_filtered2, true);
  // Convert to PCL data type
  pcl_conversions::toPCL(*table, *cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered2);
  sub_folder1 = folder1+"/table_pc/table_pc_"+frame_str+".pcd";
  pcl::io::savePCDFile (sub_folder1.c_str(), *cloud_filtered2, true);
  // folder.str("");
  // folder << "/home/omari/Datasets/test" << N.str() << std::string("/table_") << frame_str << std::string(".pcd");
  // pcl::io::savePCDFile (folder.str(), *cloud_filtered2, true);
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
    right_g = msg;
}

void leftGripperCallback (const baxter_core_msgs::EndEffectorState& msg)
{
    left_g = msg;
}

int main(int argc, char **argv)
{
  // check if user entered folder number
  if (argc < 2)
  {
    PCL_WARN("Please set the folder (e.g. rosrun pcl_functions save_data 6)\n", argv[0]);
    exit (1);
  }

  // check if folder exists so we don't over write it
  folder1 = "/home/omari/Datasets/test"+std::string(argv[1]);
  if( stat( folder1.c_str(), &info ) != 0 )
  {
      printf( "creating %s\n", folder1.c_str() );
      boost::filesystem::create_directories(folder1.c_str());
      sub_folder1 = folder1+ "/tracking";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/tabletop_pc";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/table_pc";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/kinect_rgb";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/LH_rgb";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/RH_rgb";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/LH_data";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/RH_data";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
      sub_folder1 = folder1+ "/clusters";
      printf( "creating %s\n", sub_folder1.c_str() );
      boost::filesystem::create_directories(sub_folder1.c_str());
  }
  else if( info.st_mode & S_IFDIR )
  {
    printf( "%s exists\n", folder1.c_str() );
    exit (1);
  }
  else
  {
      printf( "creating %s\n", folder1.c_str() );
      boost::filesystem::create_directories(folder1.c_str());
  }

  // Initialize node and subscribers
  N.str("");
  N << argv[1];
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("left");
  cv::namedWindow("right");
  cv::namedWindow("kinect");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("/cameras/left_hand_camera/image", 1, left_imageCallback);
  image_transport::Subscriber sub2 = it.subscribe("/cameras/right_hand_camera/image", 1, right_imageCallback);
  image_transport::Subscriber sub3 = it.subscribe("/kinect2_1/qhd/image_color", 1, kinect_imageCallback);
  ros::Subscriber sub4 = nh.subscribe ("/table_pointcloud_1", 1, cloud_table);
  ros::Subscriber sub5 = nh.subscribe ("/tabletop_pointcloud_1", 1, cloud_cb);
	ros::Subscriber sub6 = nh.subscribe("/robot/limb/left/endpoint_state", 1, leftCallback);
	ros::Subscriber sub7 = nh.subscribe("/robot/limb/right/endpoint_state", 1, rightCallback);
	ros::Subscriber sub8 = nh.subscribe("/robot/end_effector/right_gripper/state", 1, rightGripperCallback);
	ros::Subscriber sub9 = nh.subscribe("/robot/end_effector/left_gripper/state", 1, leftGripperCallback);
  ros::spin();
  cv::destroyWindow("left");
  cv::destroyWindow("right");
  cv::destroyWindow("kinect");
}
