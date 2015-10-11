#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/features/grsd.h>
#include <pcl/features/esf.h>

bool flag = false;
// Plotter object.
pcl::visualization::PCLHistogramVisualizer viewer;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //ESFS
  // Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the ESF descriptor.
	pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.
  // convert from pointcloud2 to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  // convert from PCLpointcloud2 to XYZ and RGB
  pcl::fromPCLPointCloud2 (pcl_pc2, *object);
	// ESF estimation object.
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	esf.setInputCloud(object);
	esf.compute(*descriptor);

  // //VFH
  // // Clouds for storing everything.
	// pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// // pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
	// // Object for storing the GRSD descriptors for each point.
	// pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::PointCloud<pcl::GRSDSignature21>());
  //
  // // convert from pointcloud2 to PCL
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  // // convert from PCLpointcloud2 to XYZ and RGB
  // pcl::fromPCLPointCloud2 (pcl_pc2, *object);
  //
	// // Estimate the normals.
	// pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	// normalEstimation.setInputCloud(object);
	// normalEstimation.setRadiusSearch(0.03);
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	// normalEstimation.setSearchMethod(kdtree);
	// normalEstimation.compute(*normals);
  //
  // // Estimate VFH descriptor.
  // pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  // vfh.setInputCloud(object);
  // vfh.setInputNormals(normals);
  // vfh.setSearchMethod(kdtree);
  // vfh.setNormalizeBins(true);
  // vfh.setNormalizeDistance(false);
  // vfh.compute(*descriptor);

  if(flag)
  {
    viewer.updateFeatureHistogram(*descriptor, 308);
  }
  else
  {
  	// We need to set the size of the descriptor beforehand.
  	viewer.addFeatureHistogram(*descriptor, 308);
    flag=1;
  }
  viewer.spinOnce(10);



  //#################################################//
  // Opencv
  //#################################################//
  // std::cout << "pointcloud received..." << pc_num << std::endl;
  // pc_num++;
  // if(flag)
  // {
  //   visualizer->updatePointCloud(rgb_cloud, cloudName);
  //   visualizer->saveScreenshot(ss.str());
  // }
  // else
  // {
  //   // pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer 2"));
  //   visualizer->addPointCloud(rgb_cloud, cloudName);
  //   visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
  //   visualizer->initCameraParameters();
  //   visualizer->setBackgroundColor(0, 0, 0);
  //   visualizer->setSize(800, 600);
  //   visualizer->setPosition(0, 0);
  //   // visualizer->setSize(color.cols, color.rows);
  //   visualizer->setShowFPS(true);
  //   visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
  //   flag=1;
  // }
  // visualizer->spinOnce(10);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "histogram_viewer");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_pointcloud", 1, cloud_cb);
  // ros::Subscriber sub2 = nh.subscribe ("/table_pointcloud", 1, table_cb);

  //#################################################//
  // RVIZ visualizer
  //#################################################//
  // pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud/", 30);
  //#################################################//

  // Spin
  ros::spin ();
}
