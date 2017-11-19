#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>

using namespace pcl;

class Back_end
{
public:
	Back_end()
	{
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp_.setMaxCorrespondenceDistance (0.05);
		// Set the maximum number of iterations (criterion 1)
		icp_.setMaximumIterations (50);
		// Set the transformation epsilon (criterion 2)
		icp_.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp_.setEuclideanFitnessEpsilon (1);

		ready_ = false;
	}

	void callback(const PointCloud<PointXYZ>::ConstPtr& msg)
	{
		if (ready_)
		{
			icp_.setInputSource (msg);
			icp_.setInputTarget (&target_);
			icp_.align(&target_);
			Eigen::Matrix4f transformation = icp_.getFinalTransformation ();
			ROS_INFO_STREAM(transformation);
		}		
		target_ = PointCloud<PointXYZ>(*msg);		
	}
private:
	IterativeClosestPoint<PointXYZ, PointXYZ> icp_;
	PointCloud<PointXYZ> target_;
	bool ready_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "back_end");
	ros::NodeHandle node;

	Back_end back_end;

	ros::Subscriber sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("Points", 10, &Back_end::callback, &back_end);

	ros::spin();
	return 0;
}