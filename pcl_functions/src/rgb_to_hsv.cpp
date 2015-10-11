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
ros::Publisher pub2;
bool flag = true;
int frame = 0;
double PI = 3.14159265;
bool cylinder_flag = true;
pcl::PointCloud<pcl::PointXYZRGB> cylinder;
pcl::PointCloud<pcl::PointXYZRGB> cylinder2;
sensor_msgs::PointCloud2 output;
// float x_1 = -0.39;
// float x_2 = 0.2;
// float y_1 = 0.23;
// float y_2 = 0.79;
// float z_1 = 0.2;
// float z_2 = 0.87;
// float theta =  5.044;

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PCLPointCloud2 cloud_filtered;

  // convert from pointcloud2 to PCL
  // pcl_conversions::toPCL(*cloud_msg,pcl_pc2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromPCLPointCloud2(*cloud,*hsv_cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered);
  std::cout << "/* point cloud size */" << cloud_filtered->size() << std::endl;

  //####################################################################################    converting rgb to hsv -> xyz

  double      min, max, delta, r, g, b, h, s, v, x, y, z, dis_r, dis_g, dis_b, dis_w;

  for (int pit = 0; pit < hsv_cloud->size() ; pit++)
  {
    //   std::cout << "point " << pit << " : " << cloud_filtered->points[pit] << std::endl
    //   h=0;
    //   s=0;
    //   v=0;
      r = hsv_cloud->points[pit].r;
      g = hsv_cloud->points[pit].g;
      b = hsv_cloud->points[pit].b;

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

      x = s*cos(h*PI/180.0);
      y = s*sin(h*PI/180.0);
      z = v;
      hsv_cloud->points[pit].x = x;
      hsv_cloud->points[pit].y = y;
      hsv_cloud->points[pit].z = z;
      // hsv_cloud->points[pit].r = (x+1.0)*255/2;
      // hsv_cloud->points[pit].g = (y+1.0)*255/2;
      // hsv_cloud->points[pit].b = z*255;


      // the actual test
      dis_r = sqrt(pow(x-0.38177621,2.0) + pow(y-0.10490212,2.0) + pow(z-0.58518914,2.0)); //red
      dis_g = sqrt(pow(x+0.13,2.0) + pow(y-0.44343874,2.0) + pow(z-0.3631243,2.0)); //green
      dis_b = sqrt(pow(x+.13,2.0) + pow(y+0.44343874,2.0) + pow(z-.3631243,2.0)); //blue
      dis_w = sqrt(pow(x-0.0,2.0) + pow(y-0.0,2.0) + pow(z-0.7358909,2.0)); //white
      // mydist = {dis_r,dis_g,dis_b,dis_w};
      // std::cout << "/* message */" << mydist << std::endl;
      if (dis_r<dis_b && dis_r<dis_g && dis_r<dis_w)
      {
        cloud_filtered->points[pit].r = 255;
        cloud_filtered->points[pit].g = 0;
        cloud_filtered->points[pit].b = 0;
      }
      else if(dis_b<dis_r && dis_b<dis_g && dis_b<dis_w)
      {
        cloud_filtered->points[pit].r = 0;
        cloud_filtered->points[pit].g = 0;
        cloud_filtered->points[pit].b = 255;
      }
      else if(dis_w<dis_r && dis_w<dis_g && dis_w<dis_b)
      {
        cloud_filtered->points[pit].r = 255;
        cloud_filtered->points[pit].g = 255;
        cloud_filtered->points[pit].b = 255;
      }
      else if(dis_g<dis_r && dis_g<dis_w && dis_g<dis_b)
      {
        cloud_filtered->points[pit].r = 0;
        cloud_filtered->points[pit].g = 255;
        cloud_filtered->points[pit].b = 0;
      }
  }

  //####################################################################################    remove outlayers
  // pcl::toPCLPointCloud2(*hsv_cloud,*cloud);
  // // //  Perform the actual filtering Remove outlayer, very slow !
  // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlayer2;
  // outlayer2.setInputCloud (cloudPtr);
  // outlayer2.setMeanK (8);
  // outlayer2.setStddevMulThresh (.1);
  // outlayer2.filter (*cloud);
  // pcl::fromPCLPointCloud2(*cloud,*hsv_cloud);


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
      cylinder.points[i].z = -.02;
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
      cylinder2.points[i].z = 1.02;
      cylinder2.points[i].r = 0;
      cylinder2.points[i].g = 0;
      cylinder2.points[i].b = 255;
    }
    cylinder_flag = false;
  }
  *hsv_cloud += cylinder;
  *hsv_cloud += cylinder2;




  //####################################################################################    publishing results
  pcl::toPCLPointCloud2(*hsv_cloud,*cloud);
  // Convert to ROS data type
  pcl_conversions::fromPCL(*cloud, output);
  // Publish the data
  pub.publish (output);

  pcl::toPCLPointCloud2(*cloud_filtered,*cloud);
  // Convert to ROS data type
  pcl_conversions::fromPCL(*cloud, output);
  // Publish the data
  pub2.publish (output);
  // pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rgb_to_hsv");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_pointcloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/hsv_pointcloud", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/colored_pointcloud", 1);

  std::cout << "filter + segmentation is running..." << std::endl;
  std::cout << "waiting for /filtered_pointcloud from kinect2_publisher..." << std::endl;
  // Spin
  ros::spin ();
}
