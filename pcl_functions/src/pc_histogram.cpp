#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
// #include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/common/transforms.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
// #include "cob_3d_features/organized_normal_estimation_omp.h"

ros::Publisher pub;
bool flag = true;
int frame = 0;

float x_1 = -0.42;
float x_2 = 0.19;
float y_1 = 0.08;
float y_2 = 0.96;
float z_1 = 0.24;
float z_2 = 0.77;
float theta =  5.044;
float phi = 0.0;
float psi = 0.0;
float pc = 0.0;
float clusters = 0.0;
float table = 0.0;
float table_param = 0.015;

void Callback_pc(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "point cloud = " << *msg << std::endl;
  pc = msg->data;
}

void Callback_clusters(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "clusters = " << *msg << std::endl;
  clusters = msg->data;
}

void Callback_table(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "table = " << *msg << std::endl;
  table = msg->data;
}

void Callback_theta(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "theta = " << *msg << std::endl;
  theta = msg->data;
}

void Callback_x1(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "x1 = " << *msg << std::endl;
  x_1 = msg->data;
}

void Callback_x2(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "x2 = " << *msg << std::endl;
  x_2 = msg->data;
}

void Callback_y1(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "y1 = " << *msg << std::endl;
  y_1 = msg->data;
}

void Callback_y2(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "y2 = " << *msg << std::endl;
  y_2 = msg->data;
}

void Callback_z1(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "z1 = " << *msg << std::endl;
  z_1 = msg->data;
}

void Callback_z2(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "z2 = " << *msg << std::endl;
  z_2 = msg->data;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if(flag)
  {
      flag = false;
      std::cout << "/original_pointcloud received..." << std::endl;
  }


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  // convert from pointcloud2 to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  //####################################################################################   intial filtering
  // Create the filtering object
  // Filter Z
  pcl::PassThrough<pcl::PCLPointCloud2> pass_init;
  pass_init.setInputCloud (cloudPtr);
  pass_init.setFilterFieldName ("z");
  pass_init.setFilterLimits (0, 1.2);
  pass_init.filter (*cloud);

  //####################################################################################    reduce the point cloud
  // Perform point cloud reduction makes it faster and easier to process
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (.01, .01, .01);
  // sor.filter (*cloud);


  //####################################################################################    rotation
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
  transform_2.rotate (Eigen::AngleAxisf (phi, Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (psi, Eigen::Vector3f::UnitZ()));
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromPCLPointCloud2(*cloud,*transformed_cloud);
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_2);
  pcl::toPCLPointCloud2(*transformed_cloud,*cloud);

  if (pc)
  {
    z_1 = z_1;
  }
  else
  {
    //####################################################################################    filter xyz
    // Create the filtering object
    // Filter Z
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_1, z_2);
    pass.filter (*cloud);
    // Filter x
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_1, x_2);
    pass.filter (*cloud);
    // Filter y
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_1, y_2);
    pass.filter (*cloud);


    //####################################################################################    threshold xyz and build a histogram
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_histogram (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::fromPCLPointCloud2(*cloud,*cloud_histogram);
    for (size_t i = 0; i < cloud_histogram->points.size (); ++i)
    {
      cloud_histogram->points[i].x = float(int(cloud_histogram->points[i].x*500))/1000;
      cloud_histogram->points[i].y = float(int(cloud_histogram->points[i].y*500))/1000;
      cloud_histogram->points[i].z = float(int(cloud_histogram->points[i].z*500))/1000;
    }
    pcl::toPCLPointCloud2(*cloud_histogram,*cloud);


    //####################################################################################    remove outlayers
    // pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
    // // Perform the actual filtering Remove outlayer, very slow !
    // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer;
    // outlayer.setInputCloud (cloudPtr);
    // outlayer.setMeanK (20);
    // outlayer.setStddevMulThresh (.1);
    // outlayer.filter (*cloud);
    // pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);


    //####################################################################################    distance cluster
    // Creating the KdTree object for the search method of the extraction
    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    // tree->setInputCloud (cloud_filtered);
    //
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // ec.setClusterTolerance (0.02); // 1cm
    // ec.setMinClusterSize (600);
    // ec.setMaxClusterSize (25000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud_filtered);
    // ec.extract (cluster_indices);
    //
    // int j = 0;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //     float r_all = 0;
    //     float g_all = 0;
    //     float b_all = 0;
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    //     // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //     // {
    //     //   r_all += float(cloud_filtered->points[*pit].r);
    //     //   g_all += float(cloud_filtered->points[*pit].g);
    //     //   b_all += float(cloud_filtered->points[*pit].b);
    //     //   cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    //     // }
    //     // int r = int(r_all/cloud_cluster->points.size ());
    //     // int g = int(g_all/cloud_cluster->points.size ());
    //     // int b = int(b_all/cloud_cluster->points.size ());
    //     //
    //     // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //     // {
    //     //   cloud_filtered->points[*pit].r = r;
    //     //   cloud_filtered->points[*pit].g = g;
    //     //   cloud_filtered->points[*pit].b = b;
    //     // }
    //     cloud_cluster->width = cloud_cluster->points.size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     j++;
    // }
    // frame++;
    //#################################################################################### publish

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // if (clusters)
    //     *final_cloud += *cloud_filtered;
    // if (table)
    //     *final_cloud += *cloud_plane;
    // pcl::toPCLPointCloud2(*final_cloud,*cloud);
  }
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/original_pointcloud", 1, cloud_cb);
  ros::Subscriber sub_x1 = nh.subscribe("x1", 1000, Callback_x1);
  ros::Subscriber sub_x2 = nh.subscribe("x2", 1000, Callback_x2);
  ros::Subscriber sub_y1 = nh.subscribe("y1", 1000, Callback_y1);
  ros::Subscriber sub_y2 = nh.subscribe("y2", 1000, Callback_y2);
  ros::Subscriber sub_z1 = nh.subscribe("z1", 1000, Callback_z1);
  ros::Subscriber sub_z2 = nh.subscribe("z2", 1000, Callback_z2);
  ros::Subscriber sub_pc = nh.subscribe("pc", 1000, Callback_pc);
  ros::Subscriber sub_clusters = nh.subscribe("clusters", 1000, Callback_clusters);
  ros::Subscriber sub_table = nh.subscribe("table", 1000, Callback_table);
  ros::Subscriber sub_theta = nh.subscribe("theta", 1000, Callback_theta);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);

  std::cout << "filter + segmentation is running..." << std::endl;
  std::cout << "waiting for /original_pointcloud from kinect2_publisher..." << std::endl;
  // Spin
  ros::spin ();
}
