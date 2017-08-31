#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector> 
#include <Eigen/Dense>
#include "geometry_msgs/Point32.h"
#include "underwater_slam/PointDetection.h"

std::vector<Eigen::Vector3d> point_list;

class Point_server
{
public:
	Point_server(){}
	~Point_server(){}
	bool dectect_point(underwater_slam::PointDetection::Request &, 
						underwater_slam::PointDetection::Response &res);
private:
	tf::TransformListener listener_;
	tf::StampedTransform transform_;
};

bool Point_server::dectect_point(underwater_slam::PointDetection::Request &req, 
								underwater_slam::PointDetection::Response &res)
{
	listener_.waitForTransform("/world", "/girona500",
			ros::Time(0), ros::Duration(1.0));
	listener_.lookupTransform("/world", "/girona500",
			ros::Time(0), transform_);
	Eigen::Vector3d pos(transform_.getOrigin().getX(),transform_.getOrigin().getY(),transform_.getOrigin().getZ());
	double roll, yaw, pitch;
	transform_.getBasis().getEulerYPR(yaw, pitch, roll);
	Eigen::Vector3d temp;
	double distence;
	geometry_msgs::Point32 point;

	for(std::vector<Eigen::Vector3d>::iterator it = point_list.begin(); it != point_list.end(); it++)
	{
		temp = *it-pos;
		distence = temp.norm();
		if (distence < 8)
		{
			tf::Vector3 point_out(temp(0)*cos(yaw)+temp(1)*sin(yaw), temp(1)*cos(yaw)-temp(0)*sin(yaw), temp(2));
			point.x = point_out.getX();
			point.y = point_out.getY();
			point.z = point_out.getZ();
			res.res.points.push_back(point);
		}
	}
	return true;
}

void point_list_init()
{
	point_list.push_back(Eigen::Vector3d(0,0,0));
	point_list.push_back(Eigen::Vector3d(-3,0,0));
	point_list.push_back(Eigen::Vector3d(1,2,0));
	point_list.push_back(Eigen::Vector3d(-3,-4,0));
	point_list.push_back(Eigen::Vector3d(-1,-3,0));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_detection");
	ros::NodeHandle node;
	Point_server server;
	point_list_init();

	ros::ServiceServer service = node.advertiseService("point_detection", &Point_server::dectect_point, &server);
	
	ROS_INFO("Ready to dectect the key points.");

	ros::spin();

	return 0;
}