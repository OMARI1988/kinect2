#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

//#################################################//
// Opencv
//#################################################//
bool flag=false;
pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
const std::string cloudName = "filtered";
int  pc_num = 1;
int frame = 1;

//#################################################//
// RVIZ visualizer
//#################################################//
// // #define Z_MIN 1.0
// // #define Z_MAX 1.5
// ros::Publisher pub_cloud;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // convert from pointcloud2 to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);

  // convert from PCLpointcloud2 to XYZ only
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*xyz_cloud);

  // convert from PCLpointcloud2 to XYZ and RGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (pcl_pc2, *rgb_cloud);

  //#################################################//
  // Opencv
  //#################################################//
  std::cout << "pointcloud received..." << pc_num << std::endl;
  pc_num++;
  if(flag)
  {
    visualizer->updatePointCloud(rgb_cloud, cloudName);
    std::stringstream ss;
    if(frame<10)
    {
     ss << "/home/omari/Datasets/pointclouds/scene_00001/img000" << frame << ".png";
    }
    else if (frame<100)
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img00" << frame << ".png";
    }
    else if (frame<1000)
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img0" << frame << ".png";
    }
    else
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img" << frame << ".png";
    }
    frame++;
    visualizer->saveScreenshot(ss.str());
  }
  else
  {
    // pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer 2"));
    visualizer->addPointCloud(rgb_cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setSize(800, 600);
    visualizer->setPosition(0, 0);
    // visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    std::stringstream ss;
    if(frame<10)
    {
     ss << "/home/omari/Datasets/pointclouds/scene_00001/img000" << frame << ".png";
    }
    else if (frame<100)
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img00" << frame << ".png";
    }
    else if (frame<1000)
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img0" << frame << ".png";
    }
    else
    {
      ss << "/home/omari/Datasets/pointclouds/scene_00001/img" << frame << ".png";
    }
    frame++;
    visualizer->saveScreenshot(ss.str());
    // visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);
    flag=1;
  }
  visualizer->spinOnce(10);
  //#################################################//



  //#################################################//
  // RVIZ visualizer
  //#################################################//
  // uint8_t r = 0;
  // uint8_t g = 0;
  // uint8_t b = 0;
  // int32_t rgb = (r << 16) | (g << 8) | b;
  // // remove points with a certain Z range
  // // for (size_t i = 0; i < rgb_cloud->points.size (); ++i)
  // // {
  // //   if ((rgb_cloud->points[i].z > Z_MAX) or (rgb_cloud->points[i].z < Z_MIN))
  // //   {
  // //     rgb_cloud->points[i].rgb = rgb;
  // //   }
  // // }
  // pcl::toPCLPointCloud2 (*rgb_cloud, pcl_pc2);
  // sensor_msgs::PointCloud2 ros_pc2;
  // pcl_conversions::fromPCL(pcl_pc2, ros_pc2);
  // ros_pc2.header.frame_id="camera_optical_frame";
  // pub_cloud.publish(ros_pc2);
  //#################################################//
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_viewer");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_pointcloud", 1, cloud_cb);

  //#################################################//
  // RVIZ visualizer
  //#################################################//
  // pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud/", 30);
  //#################################################//

  // Spin
  ros::spin ();
}
