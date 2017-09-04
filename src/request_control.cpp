#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include "tf/transform_datatypes.h"
#include "underwater_slam/RequireControl.h"
#include "underwater_sensor_msgs/DVL.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

class Control_server
{
public:
	Control_server()
	{
		ready_ = false;
	}
	~Control_server(){}
	bool request_control(underwater_slam::RequireControl::Request &req, 
						underwater_slam::RequireControl::Response &res);
	void update_imu(const sensor_msgs::Imu::ConstPtr&);
	void update_dvl(const underwater_sensor_msgs::DVL::ConstPtr&);
private:
	geometry_msgs::Vector3 vel_;
	geometry_msgs::Quaternion q_;
	boost::mutex mutex_;
	bool ready_;
};

void Control_server::update_dvl(const underwater_sensor_msgs::DVL::ConstPtr& dvl)
{
	mutex_.lock();
	vel_.x = dvl->bi_x_axis;
	vel_.y = dvl->bi_y_axis;
	vel_.z = dvl->bi_z_axis;
	ready_ = true;
	mutex_.unlock();
}

void Control_server::update_imu(const sensor_msgs::Imu::ConstPtr& imu)
{
	mutex_.lock();
	q_ = imu->orientation;
	ready_ = true;
	mutex_.unlock();
}


bool Control_server::request_control(underwater_slam::RequireControl::Request &req, 
						underwater_slam::RequireControl::Response &res)
{
	while(!ready_)
		;
	mutex_.lock();
	res.orientation = q_;
	res.linear_velocity = vel_;
	res.yaw = tf::getYaw(q_);
	mutex_.unlock();

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "request_control");
	ros::NodeHandle node;
	Control_server server;

  ros::Subscriber sub_imu = node.subscribe("g500/imu", 1, &Control_server::update_imu, &server);
  ros::Subscriber sub_dvl = node.subscribe("g500/dvl", 1, &Control_server::update_dvl, &server);
	ros::ServiceServer service = node.advertiseService("request_control", &Control_server::request_control, &server);
	
	ROS_INFO("Ready to request the control.");

	ros::spin();

	return 0;
}