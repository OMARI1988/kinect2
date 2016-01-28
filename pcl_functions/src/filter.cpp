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
ros::Publisher pub2;
bool flag = true;
int frame = 0;

float x_1 = -0.55;
float x_2 = 0.33;
float y_1 = -.286;
float y_2 = 0.977;
float z_1 = 0.506;
float z_2 = 0.883;
float theta = 0;
float phi = 0.0;
float psi = 0.0;
float pc = 0.0;
float clusters = 1.0;
float table = 0.0;
float table_param = 0.015;
float leaf = 0.005;
sensor_msgs::PointCloud2 output;
bool save = false;
std::stringstream folder;

void Callback_save(const std_msgs::Float64::ConstPtr& msg)
{
  frame+=1;
  folder.str("");
  if(frame<10)
  {
    folder << "/home/omari/Datasets/Static_Scenes/scene_000" << frame;
  }
  else if (frame<100)
  {
    folder << "/home/omari/Datasets/Static_Scenes/scene_00" << frame;
  }
  else if (frame<1000)
  {
    folder << "/home/omari/Datasets/Static_Scenes/scene_0" << frame;
  }
  else
  {
    folder << "/home/omari/Datasets/Static_Scenes/scene_" << frame;
  }
  save=true;
}

void Callback_leaf(const std_msgs::Float64::ConstPtr& msg)
{
  std::cout << "leaf size = " << *msg << std::endl;
  leaf = msg->data;
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
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*cloud_msg,pcl_pc2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  //####################################################################################   intial filtering
  // Create the filtering object
  // Filter Z
  // pcl::PassThrough<pcl::PCLPointCloud2> pass_init;
  // pass_init.setInputCloud (cloudPtr);
  // pass_init.setFilterFieldName ("z");
  // pass_init.setFilterLimits (0, 20);
  // pass_init.filter (*cloud);

  //####################################################################################    reduce the point cloud
  // Perform point cloud reduction makes it faster and easier to process
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (leaf, leaf, leaf);
  // sor.filter (*cloud);


  //####################################################################################    rotation
  // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
  // // transform_2.rotate (Eigen::AngleAxisf (phi, Eigen::Vector3f::UnitY()));
  // // transform_2.rotate (Eigen::AngleAxisf (psi, Eigen::Vector3f::UnitZ()));
  // // Executing the transformation
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // pcl::fromPCLPointCloud2(*cloud,*transformed_cloud);
  // // You can either apply transform_1 or transform_2; they are the same
  // pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_2);
  // pcl::toPCLPointCloud2(*transformed_cloud,*cloud);
  //
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

  //####################################################################################    detect table
  // convert from PCLpointcloud2 to XYZRGB only
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);
  // Create the segmentation object for the planar model
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (20);
  seg.setDistanceThreshold (table_param);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      // std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    }

    //####################################################################################    remove outlayers
    pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
    // Perform the actual filtering Remove outlayer, very slow !
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer;
    outlayer.setInputCloud (cloudPtr);
    outlayer.setMeanK (20);
    outlayer.setStddevMulThresh (.001);
    outlayer.filter (*cloud);
    // pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);

    //#################################################################################### get the color of the cloud plane
    // float r_all = 0;
    // float g_all = 0;
    // float b_all = 0;
    //
    // for (int pit = 0; pit < cloud_plane->size() ; pit++)
    // {
    //   r_all += float(cloud_plane->points[pit].r);
    //   g_all += float(cloud_plane->points[pit].g);
    //   b_all += float(cloud_plane->points[pit].b);
    // }
    // int r = int(r_all/cloud_plane->points.size ());
    // int g = int(g_all/cloud_plane->points.size ());
    // int b = int(b_all/cloud_plane->points.size ());
    // for (int pit = 0; pit < cloud_plane->size() ; pit++)
    // {
    //   cloud_plane->points[pit].r = r;
    //   cloud_plane->points[pit].g = g;
    //   cloud_plane->points[pit].b = b;
    // }


    //#################################################################################### publish plane
    pcl::toPCLPointCloud2(*cloud_plane,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    pub2.publish (output);


    pcl::toPCLPointCloud2(*cloud_filtered,*cloud);


  //
  //
  //   //####################################################################################    distance cluster
  //   // Creating the KdTree object for the search method of the extraction
  //   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  //   tree->setInputCloud (cloud_filtered);
  //
  //   std::vector<pcl::PointIndices> cluster_indices;
  //   pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  //   ec.setClusterTolerance (0.02); // 1cm
  //   ec.setMinClusterSize (600);
  //   ec.setMaxClusterSize (25000);
  //   ec.setSearchMethod (tree);
  //   ec.setInputCloud (cloud_filtered);
  //   ec.extract (cluster_indices);
  //
  //   int j = 0;
  //   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  //   {
  //       float r_all = 0;
  //       float g_all = 0;
  //       float b_all = 0;
  //       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  //       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //       // {
  //       //   r_all += float(cloud_filtered->points[*pit].r);
  //       //   g_all += float(cloud_filtered->points[*pit].g);
  //       //   b_all += float(cloud_filtered->points[*pit].b);
  //       //   cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
  //       // }
  //       // int r = int(r_all/cloud_cluster->points.size ());
  //       // int g = int(g_all/cloud_cluster->points.size ());
  //       // int b = int(b_all/cloud_cluster->points.size ());
  //       //
  //       // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //       // {
  //       //   cloud_filtered->points[*pit].r = r;
  //       //   cloud_filtered->points[*pit].g = g;
  //       //   cloud_filtered->points[*pit].b = b;
  //       // }
  //       cloud_cluster->width = cloud_cluster->points.size ();
  //       cloud_cluster->height = 1;
  //       cloud_cluster->is_dense = true;
  //
  //     // save clusters point cloud at home directory
  //     //   std::stringstream ss;
  //     //   if(frame<10)
  //     //   {
  //     //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_000" << frame << "_cloud_cluster_" << j << ".pcd";
  //     //   }
  //     //   else if (frame<100)
  //     //   {
  //     //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_00" << frame << "_cloud_cluster_" << j << ".pcd";
  //     //   }
  //     //   else if (frame<1000)
  //     //   {
  //     //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_0" << frame << "_cloud_cluster_" << j << ".pcd";
  //     //   }
  //     //   elsepc
  //     //   {
  //     //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_" << frame << "_cloud_cluster_" << j << ".pcd";
  //     //   }
  //       j++;
    // }
  //   frame++;
  //   //#################################################################################### publish
  //
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   if (clusters)
  //       *final_cloud += *cloud_filtered;
  //   if (table)
  //       *final_cloud += *cloud_plane;
  //   pcl::toPCLPointCloud2(*final_cloud,*cloud);
  // }
  // Convert to ROS data type
  // if (save)
  // {
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   pcl::fromPCLPointCloud2(*cloud,*cloud_rgb);
  //   std::stringstream file;
  //   file << folder.str() << "/pc_original.pcd";
  //   std::cout << "saving " << file.str() << std::endl;
  //   save=false;
  //   pcl::io::savePCDFileASCII (file.str(), *cloud_rgb, true);
  //   // file.clear();
  //   // std::cout << "original pointcloud saved " << frame << std::endl;
  // }

  pcl_conversions::fromPCL(*cloud, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // make sure not to overwrite my dataset
  // DIR *dir;
  // struct dirent *ent;
  // if ((dir = opendir ("/home/omari/Datasets/Static_Scenes/")) != NULL) {
  //   /* print all the files and directories within directory */
  //   while ((ent = readdir (dir)) != NULL) {
  //     frame+=1;
  //   }
  //   closedir (dir);
  // } else {
  //   /* could not open directory */
  //   perror ("");
  //   return EXIT_FAILURE;
  // }
  // frame-=2;
  // std::cout << "Last scene saved == " << frame << std::endl;


  // Initialize ROS
  ros::init (argc, argv, "filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/qhd/points", 1, cloud_cb);
  ros::Subscriber sub_x1 = nh.subscribe("x1", 1000, Callback_x1);
  ros::Subscriber sub_x2 = nh.subscribe("x2", 1000, Callback_x2);
  ros::Subscriber sub_y1 = nh.subscribe("y1", 1000, Callback_y1);
  ros::Subscriber sub_y2 = nh.subscribe("y2", 1000, Callback_y2);
  ros::Subscriber sub_z1 = nh.subscribe("z1", 1000, Callback_z1);
  ros::Subscriber sub_z2 = nh.subscribe("z2", 1000, Callback_z2);
  ros::Subscriber sub_pc = nh.subscribe("pc", 1000, Callback_pc);
  ros::Subscriber sub_leaf = nh.subscribe("leaf", 1000, Callback_leaf);
  ros::Subscriber sub_clusters = nh.subscribe("clusters", 1000, Callback_clusters);
  ros::Subscriber sub_table = nh.subscribe("table", 1000, Callback_table);
  ros::Subscriber sub_theta = nh.subscribe("theta", 1000, Callback_theta);
  // ros::Subscriber sub_save = nh.subscribe("save", 1000, Callback_save);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/table_pointcloud", 1);

  std::cout << "filter + segmentation is running..." << std::endl;
  std::cout << "waiting for /original_pointcloud from kinect2_publisher..." << std::endl;
  // Spin
  ros::spin ();
}
