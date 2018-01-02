#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>
#include "sensor_msgs/LaserScan.h"
#include <boost/thread/mutex.hpp>
#include "boost/numeric/ublas/matrix.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "nav_msgs/Path.h"
#include "underwater_slam/RequireControl.h"
#include <pcl_ros/point_cloud.h>
#include "Node.hpp"
#include "FrontEnd.hpp"
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "slam");
	ros::NodeHandle node;

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   	ros::console::notifyLoggerLevelsChanged();
	}

	FrontEnd front_end;
	underwater_slam::RequireControl control_srv;

	ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("g500/multibeam", 10, &FrontEnd::laser_callback, &front_end);
  ros::ServiceClient control_client = node.serviceClient<underwater_slam::RequireControl>("request_control");
  ros::Publisher pub_points = node.advertise<pcl::PointCloud<pcl::PointXYZ>> ("Points", 1);

	ros::Rate loop_rate(60);

	Eigen::Vector2d pos = Eigen::Vector2d::Zero();
	Eigen::Vector2d start_pos = Eigen::Vector2d::Zero();

	ros::Time before, after;
  ros::Duration duration;
  double det_t;
  before = ros::Time::now();
  nav_msgs::Path path_rst;

	while(ros::ok())
	{
		Eigen::Vector2d velocity;
		Eigen::Vector2d det;
		Eigen::Quaternionf q;
		double yaw;

		after = ros::Time::now();
    duration = after - before;
    before = after;
    det_t = duration.toSec();

		if(!control_client.call(control_srv))
  	{	
    	ROS_ERROR("Failed to call service");
    	return 1;
  	}
		
		yaw = control_srv.response.yaw;
		q.w() = control_srv.response.orientation.w;
		q.vec() << control_srv.response.orientation.x, control_srv.response.orientation.y, control_srv.response.orientation.z;
		velocity << control_srv.response.linear_velocity.x, control_srv.response.linear_velocity.y;
		det = velocity*det_t;

		pos(0) = det(0)*cos(yaw) - det(1)*sin(yaw);
		pos(1) = det(0)*sin(yaw) + det(1)*cos(yaw);

		start_pos += pos;

		front_end.set_pos(pos);
		front_end.set_q(q);

		front_end.lock();
    if (front_end.ready())
    {
    	front_end.sented();
    	pub_points.publish(front_end.get_node().cloud);

    	// add node to graph 

    	ROS_DEBUG_STREAM("POS: " << start_pos);
    	ROS_DEBUG_STREAM("Node" << front_end.get_node().seq << " sented");
    }
    front_end.unlock();

    ros::spinOnce();
  	loop_rate.sleep();
	}

	return 0;
}	