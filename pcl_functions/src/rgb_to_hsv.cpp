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
#include <math.h>       /* sin */
// #include "cob_3d_features/organized_normal_estimation_omp.h"

ros::Publisher pub;
bool flag = true;
int frame = 0;
double PI = 3.14159265;
bool cylinder_flag = true;
pcl::PointCloud<pcl::PointXYZRGB> cylinder;
pcl::PointCloud<pcl::PointXYZRGB> cylinder2;

//####################################################################################
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
  // pcl_conversions::toPCL(*cloud_msg,pcl_pc2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  //####################################################################################    reduce the point cloud
  // Perform point cloud reduction makes it faster and easier to process
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (*cloud);

  //####################################################################################    filter xyz
  // Create the filtering object
  // Filter Z
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.2);
  pass.filter (*cloud);
  // Filter x
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.3, 0.3);
  pass.filter (*cloud);
  // Filter y
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.3, 0.3);
  pass.filter (*cloud);


  //####################################################################################    detect table
  // convert from PCLpointcloud2 to XYZRGB only
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);
  // pcl::copyPointCloud(*cloud_filtered2,*cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
    //   pcl::copyPointCloud(*cloud_filtered,*final_cloud_filtered);
      extract.setNegative (true);
      extract.filter (*cloud_filtered);
  }
  //####################################################################################    reduce the point cloud
  pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
  // Perform point cloud reduction makes it faster and easier to process
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (.004, .004, .004);
  sor.filter (*cloud);

  //####################################################################################    remove outlayers
  // Perform the actual filtering Remove outlayer, very slow !
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer;
  outlayer.setInputCloud (cloudPtr);
  outlayer.setMeanK (10);
  outlayer.setStddevMulThresh (.1);
  outlayer.filter (*cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);

  //####################################################################################    converting rgb to hsv -> xyz

  double      min, max, delta, r, g, b, h, s, v;
  for (int pit = 0; pit <= cloud_filtered->size() ; pit++)
  {
    //   std::cout << "point " << pit << " : " << cloud_filtered->points[pit] << std::endl
    //   h=0;
    //   s=0;
    //   v=0;
      r = cloud_filtered->points[pit].r;
      g = cloud_filtered->points[pit].g;
      b = cloud_filtered->points[pit].b;

      min = r < g ? r : g;
      min = min  < b ? min  : b;

      max = r > g ? r : g;
      max = max  > b ? max  : b;

      v = max/255.0;                                // v
      delta = max - min;
      if (delta < 0.00001)
      {
          s = 0;
          h = 0; // undefined, maybe nan?
      }
      if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
          s = (delta / max);                  // s
      } else {
          // if max is 0, then r = g = b = 0
              // s = 0, v is undefined
          s = 0.0;
          h = 0.0; //NAN;                            // its now undefined
      }
      if( r >= max )                           // > is bogus, just keeps compilor happy
          h = ( g - b ) / delta;        // between yellow & magenta
      else
      if( g >= max )
          h = 2.0 + ( b - r ) / delta;  // between cyan & yellow
      else
          h = 4.0 + ( r - g ) / delta;  // between magenta & cyan

      h *= 60.0;                              // degrees

      if( h < 0.0 )
          h += 360.0;

      if (r==0)
        if (g==0)
            if (b==0)
             h = s = v = 0.0;
      if (r==254)
        if (g==254)
          if (b==254)
            {
              h = s = 0.0;
              v = 1.0;
            }
      if (h<0.0)
        std::cout << "rgb " << pit << " : " << r << " " << g << " " << b << " " << " to hsv : " << h << " " << s << " " << v << " " << std::endl;
      if (h>360.0)
        std::cout << "rgb " << pit << " : " << r << " " << g << " " << b << " " << " to hsv : " << h << " " << s << " " << v << " " << std::endl;
      cloud_filtered->points[pit].x = s*cos(h*PI/180.0);
      cloud_filtered->points[pit].y = s*sin(h*PI/180.0);
      cloud_filtered->points[pit].z = v;
      cloud_filtered->points[pit].r = int(r);
      cloud_filtered->points[pit].g = int(g);
      cloud_filtered->points[pit].b = int(b);
  }
  // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  // {
  //   cloud_filtered->points[*pit].r = r;
  //   cloud_filtered->points[*pit].g = g;
  //   cloud_filtered->points[*pit].b = b;
  //####################################################################################    remove outlayers
  pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
  //  Perform the actual filtering Remove outlayer, very slow !
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer2;
  outlayer2.setInputCloud (cloudPtr);
  outlayer2.setMeanK (8);
  outlayer2.setStddevMulThresh (.1);
  outlayer2.filter (*cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);


  //####################################################################################    add cylinders
  if (cylinder_flag)
  {
    // Fill in the cloud data
    cylinder.width    = 500;
    cylinder.height   = 1;
    cylinder.is_dense = false;
    cylinder.points.resize (cylinder.width * cylinder.height);

    for (size_t i = 0; i < cylinder.points.size (); ++i)
    {
      cylinder.points[i].x = 1.0*cos(2*PI*i/cylinder.points.size());
      cylinder.points[i].y = 1.0*sin(2*PI*i/cylinder.points.size());
      cylinder.points[i].z = -.01;
      cylinder.points[i].r = 255;
      cylinder.points[i].g = 0;
      cylinder.points[i].b = 0;
    }
    // Fill in the cloud data
    cylinder2.width    = 500;
    cylinder2.height   = 1;
    cylinder2.is_dense = false;
    cylinder2.points.resize (cylinder2.width * cylinder2.height);

    for (size_t i = 0; i < cylinder2.points.size (); ++i)
    {
      cylinder2.points[i].x = 1.0*cos(2*PI*i/cylinder2.points.size());
      cylinder2.points[i].y = 1.0*sin(2*PI*i/cylinder2.points.size());
      cylinder2.points[i].z = 1.01;
      cylinder2.points[i].r = 0;
      cylinder2.points[i].g = 0;
      cylinder2.points[i].b = 255;
    }
    cylinder_flag = false;
  }
  *cloud_filtered += cylinder;
  *cloud_filtered += cylinder2;




  //####################################################################################    publishing results
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
  ros::init (argc, argv, "filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/original_pointcloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);

  std::cout << "filter + segmentation is running..." << std::endl;
  std::cout << "waiting for /original_pointcloud from kinect2_publisher..." << std::endl;
  // Spin
  ros::spin ();
}
