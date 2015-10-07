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
float distance_thresh = .02;
float color_thresh = .2;
float region_thresh = .2;
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

        // Get the points associated with the planar   //
    // for (int i = 0; i < small_clusters->size (); ++i)
    //   for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
    //     cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
    //
    // float r_all = 0;
    // float g_all = 0;
    // float b_all = 0;
    // for (std::vector<int>::const_iterator pit = 0; pit != cloud_plane->points.size (); ++pit)
    // {
    //   r_all += float(cloud_plane->points[*pit].r);
    //   g_all += float(cloud_plane->points[*pit].g);
    //   b_all += float(cloud_plane->points[*pit].b);
    //   cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    // }
    // int r = int(r_all/cloud_plane->points.size ());
    // int g = int(g_all/cloud_plane->points.size ());
    // int b = int(b_all/cloud_plane->points.size ());
    // for (std::vector<int>::const_iterator pit = 0; pit != cloud_plane->points.size (); ++pit)
    // {
    //   cloud_plane->points[*pit].r = r;
    //   cloud_plane->points[*pit].g = g;
    //   cloud_plane->points[*pit].b = b;
    // }surface
        extract.filter (*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
      //   pcl::copyPointCloud(*cloud_filtered,*final_cloud_filtered);
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
    }
    //
    // //#################################################################################### get the color of the cloud plane
    //
    // for (int i = 0; i < small_clusters->size (); ++i)
    //   for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
    //     cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
    //
    // float r_all = 0;
    // float g_all = 0;
    // float b_all = 0;
    // for (std::vector<int>::const_iterator pit = 0; pit != cloud_plane->points.size (); ++pit)
    // {
    //   r_all += float(cloud_plane->points[*pit].r);
    //   g_all += float(cloud_plane->points[*pit].g);
    //   b_all += float(cloud_plane->points[*pit].b);
    //   cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    // }
    // int r = int(r_all/cloud_plane->points.size ());
    // int g = int(g_all/cloud_plane->points.size ());
    // int b = int(b_all/cloud_plane->points.size ());
    // for (std::vector<int>::const_iterator pit = 0; pit != cloud_plane->points.size (); ++pit)
    // {
    //   cloud_plane->points[*pit].r = r;
    //   cloud_plane->points[*pit].g = g;
    //   cloud_plane->points[*pit].b = b;
    // }

    //####################################################################################    remove outlayers
    pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
    // Perform the actual filtering Remove outlayer, very slow !
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer;
    outlayer.setInputCloud (cloudPtr);
    outlayer.setMeanK (20);
    outlayer.setStddevMulThresh (.1);
    outlayer.filter (*cloud);
    pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);


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


    // pcl::toPCLPointCloud2(*colored_cloud,*cloud);
    //
    // //####################################################################################    distance cluster
    // // Creating the KdTree object for the search method of the extraction
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
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        float r_all = 0;
        float g_all = 0;
        float b_all = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          r_all += float(cloud_filtered->points[*pit].r);
          g_all += float(cloud_filtered->points[*pit].g);
          b_all += float(cloud_filtered->points[*pit].b);
          cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        int r = int(r_all/cloud_cluster->points.size ());
        int g = int(g_all/cloud_cluster->points.size ());
        int b = int(b_all/cloud_cluster->points.size ());

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          cloud_filtered->points[*pit].r = r;
          cloud_filtered->points[*pit].g = g;
          cloud_filtered->points[*pit].b = b;
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        // std::cout << "red " << j << " " << r_all/cloud_cluster->points.size () << std::endl;
        // std::cout << "green " << j << " " << g_all/cloud_cluster->points.size () << std::endl;
        // std::cout << "blue " << j << " " << b_all/cloud_cluster->points.size () << std::endl;
        //
        // std::cout << "PointCloud Cluster: " << j << std::endl;
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      // save clusters point cloud at home directory
      //   std::stringstream ss;
      //   if(frame<10)
      //   {
      //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_000" << frame << "_cloud_cluster_" << j << ".pcd";
      //   }
      //   else if (frame<100)
      //   {
      //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_00" << frame << "_cloud_cluster_" << j << ".pcd";
      //   }
      //   else if (frame<1000)
      //   {
      //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_0" << frame << "_cloud_cluster_" << j << ".pcd";
      //   }
      //   elsepc
      //   {
      //     ss << "/home/omari/Datasets/pointclouds/scene_00001/frame_" << frame << "_cloud_cluster_" << j << ".pcd";
      //   }
        j++;
    }
    // frame++;
    //#################################################################################### publish

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (clusters)
        *final_cloud += *cloud_filtered;
    if (table)
        *final_cloud += *cloud_plane;
    pcl::toPCLPointCloud2(*final_cloud,*cloud);
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
  ros::Subscriber sub_dis = nh.subscribe("distance", 1000, Callback_distance);
  ros::Subscriber sub_col = nh.subscribe("color", 1000, Callback_color);
  ros::Subscriber sub_reg = nh.subscribe("region", 1000, Callback_region);
  ros::Subscriber sub_cls = nh.subscribe("cluster_size", 1000, Callback_cluster_size);
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
