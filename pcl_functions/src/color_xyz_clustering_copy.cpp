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
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
// #include "cob_3d_features/organized_normal_estimation_omp.h"

ros::Publisher pub;
bool flag = true;
int frame = 0;

float x_1 = -0.39;
float x_2 = 0.2;
float y_1 = 0.23;
float y_2 = 0.79;
float z_1 = 0.2;
float z_2 = 0.87;
float theta =  5.044;
float phi = 0.0;
float psi = 0.0;
float pc = 1.0;
float clusters = 0.0;
float table = 0.0;
float table_param = 0.015;

// RGB
float distance_thresh = .009;
float color_thresh = 13.85;
float region_thresh = 4.26;
int cluster_thresh = 400;

void Callback_distance(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "distance_thresh = " << *msg << std::endl;
  distance_thresh = msg->data;
}
void Callback_color(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "color_thresh = " << *msg << std::endl;
  color_thresh = msg->data;
}
void Callback_region(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "region_thresh = " << *msg << std::endl;
  region_thresh = msg->data;
}
void Callback_cluster_size(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "cluster_thresh = " << *msg << std::endl;
  cluster_thresh = int(msg->data);
}


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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);
  std::cout << "/* hsv point cloud size */" << cloud_filtered->size() << std::endl;

  //####################################################################################    rgb xyz cluster
  // Creating the KdTree object for the search method of the extraction
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector <int>);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud_filtered);
  // reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distance_thresh);
  reg.setPointColorThreshold (color_thresh);
  reg.setRegionColorThreshold (region_thresh);
  reg.setMinClusterSize (cluster_thresh);

  std::vector <pcl::PointIndices> cluster_indices;
  reg.extract (cluster_indices);

  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  int j = 0;
  double r_all = 0;
  double g_all = 0;
  double b_all = 0;
  int r = 0;
  int g = 0;
  int b = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      r_all = 0;
      g_all = 0;
      b_all = 0;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        r_all += float(cloud_filtered->points[*pit].r);
        g_all += float(cloud_filtered->points[*pit].g);
        b_all += float(cloud_filtered->points[*pit].b);
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      }
      r = int(r_all/cloud_cluster->points.size ());
      g = int(g_all/cloud_cluster->points.size ());
      b = int(b_all/cloud_cluster->points.size ());

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_filtered->points[*pit].r = r;
        cloud_filtered->points[*pit].g = g;
        cloud_filtered->points[*pit].b = b;
      }
      // creat a single cluster for every object
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //##################################################################################//
      // Normal features computation
      //##################################################################################//
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(*cloud_cluster, *cloud_cluster_xyz);
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud_cluster_xyz);
      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);
      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.052);
      // Compute the features
      ne.compute (*cloud_normals);

      //##################################################################################//
      // Normal features histogram
      //##################################################################################//
      // // Create the PFH estimation class, and pass the input dataset+normals to it
      // pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
      // pfh.setInputCloud (cloud_cluster_xyz);
      // pfh.setInputNormals (cloud_normals);
      // // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
      // // Create an empty kdtree representation, and pass it to the PFH estimation object.
      // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
      // //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
      // pfh.setSearchMethod (tree2);
      // // Output datasets
      // pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
      // // Use all neighbors in a sphere of radius 5cm
      // // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
      // pfh.setRadiusSearch (0.02);
      // // Compute the features
      // pfh.compute (*pfhs);

      //##################################################################################//
      // fast features histogram
      //##################################################################################//
      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (cloud_cluster_xyz);
      fpfh.setInputNormals (cloud_normals);
      // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

      // Create an empty kdtree representation, and pass it to the FPFH estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);

      fpfh.setSearchMethod (tree2);

      // Output datasets
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

      // Use all neighbors in a sphere of radius 5cm
      // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
      fpfh.setRadiusSearch (0.02);

      // Compute the features
      fpfh.compute (*fpfhs);

      for (size_t i = 0; i < fpfhs->points.size(); ++i)
          std::cerr << "Signature: " << fpfhs->points[i].histogram << std::endl;


      // std::cout << "red " << j << " " << r_all/cloud_cluster->points.size () << std::endl;
      // std::cout << "green " << j << " " << g_all/cloud_cluster->points.size () << std::endl;
      // std::cout << "blue " << j << " " << b_all/cloud_cluster->points.size () << std::endl;

      // std::cout << "PointCloud Cluster: " << j << std::endl;
      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    // save clusters point cloud at home directory
      // std::stringstream ss;
      // if(frame<10)
      // {
      //   ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_000" << frame << "_cloud_cluster_" << j << ".pcd";
      // }
      // else if (frame<100)
      // {
      //   ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_00" << frame << "_cloud_cluster_" << j << ".pcd";
      // }
      // else if (frame<1000)
      // {
      //   ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_0" << frame << "_cloud_cluster_" << j << ".pcd";
      // }
      // elsepc
      // {
      //   ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_" << frame << "_cloud_cluster_" << j << ".pcd";
      // }
      j++;
  }
  // frame++;
  //#################################################################################### publish

  // pcl::toPCLPointCloud2(*colored_cloud,*cloud);
  pcl::toPCLPointCloud2(*cloud_filtered,*cloud);

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
  ros::init (argc, argv, "color_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_pointcloud", 1, cloud_cb);

  ros::Subscriber sub_dis = nh.subscribe("distance", 1000, Callback_distance);
  ros::Subscriber sub_col = nh.subscribe("color", 1000, Callback_color);
  ros::Subscriber sub_reg = nh.subscribe("region", 1000, Callback_region);
  ros::Subscriber sub_cls = nh.subscribe("cluster_size", 1000, Callback_cluster_size);
  ros::Subscriber sub_clusters = nh.subscribe("clusters", 1000, Callback_clusters);
  ros::Subscriber sub_table = nh.subscribe("table", 1000, Callback_table);
  ros::Subscriber sub_theta = nh.subscribe("theta", 1000, Callback_theta);



  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/object_clusters", 1);

  std::cout << "color segmentation is running..." << std::endl;
  std::cout << "waiting for /hsv_pointcloud from kinect2_publisher..." << std::endl;
  // Spin
  ros::spin ();
}
