#include <string>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int count = 1;

void chatterCallback(const sensor_msgs::PointCloud::ConstPtr& pcl_in)
{
	if (count %2 == 0 )
	{
		count++;
		return;
	}
	pcl::PointCloud<pcl::PointXYZ> cloud;
	std::string before = "test";
	std::string end = ".pcd";
	std::string file_name;
	cloud.width = 1200;
	cloud.height = 50;

	cloud.points.resize(cloud.width*cloud.height);
	for (int i = 0; i < pcl_in->points.size(); ++i)
	{
		cloud.points[i].x = pcl_in->points[i].x;
		cloud.points[i].y = pcl_in->points[i].y;
		cloud.points[i].z = pcl_in->points[i].z;
	}	
	std::string num = std::to_string((count+1)/2);
	file_name = before + num + end;
	std::cout << file_name << std::endl;
	pcl::io::savePCDFileASCII (file_name, cloud);
	ROS_INFO_STREAM("Save to " << file_name);
	count++;
	if (count == 50)
	{
		ros::shutdown();
	}

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "save_data");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("PointCloudConveyNow", 1, chatterCallback);

  ros::spin();

  return 0;
}