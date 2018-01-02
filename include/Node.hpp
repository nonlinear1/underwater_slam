#ifndef NODE_HPP
#define NODE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

struct Node 
{
	unsigned int seq;
	Eigen::Vector2d centre;
	Eigen::Quaternionf centre_q;
	double bound[4];
	pcl::PointCloud<pcl::PointXYZ> cloud;
};

#endif