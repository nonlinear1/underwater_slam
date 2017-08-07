#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr pre;
pcl::PointCloud<pcl::PointXYZ>::Ptr now;

void trans(boost::shared_ptr<sensor_msgs::PointCloud const> input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  for (unsigned int index = 0;index < input->points.size(); index++)
  {
    output->push_back(pcl::PointXYZ(input->points[index].x, input->points[index].y, input->points[index].z));
  }
  return;
}

// void compute (pcl::PointCloud<pcl::PointXYZ>::Ptr now, pcl::PointCloud<pcl::PointXYZ>::Ptr pre)
// {
  
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//   approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
//   approximate_voxel_filter.setInputCloud (pre);
//   approximate_voxel_filter.filter (*filtered_cloud);

//   pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

//   ndt.setTransformationEpsilon (0.01);
//   ndt.setStepSize (0.1);
//   ndt.setResolution (1.0);
//   ndt.setMaximumIterations (35);
//   ndt.setInputSource (filtered_cloud);
//   ndt.setInputTarget (now);

//   Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
//   Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
//   Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

//   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   ndt.align (*output_cloud, init_guess);

//   pcl::transformPointCloud (*pre, *output_cloud, ndt.getFinalTransformation ());
// }

void compute (pcl::PointCloud<pcl::PointXYZ>::Ptr now, pcl::PointCloud<pcl::PointXYZ>::Ptr pre)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(pre);
  icp.setInputTarget(now);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
}



void callback(boost::shared_ptr<sensor_msgs::PointCloud const> msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  trans(msg, output);
  if (!now)
  {
    now = output;
  }
  else 
  {
    pre = now;
    now = output;
    compute(now, pre);
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_reg");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("PointCloudConveyNow", 1000, callback);

  ros::spin();

  return 0;
}