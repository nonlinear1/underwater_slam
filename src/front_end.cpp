#include <ros/ros.h>
#include <Eigen/Dense>
#include "sensor_msgs/LaserScan.h"
#include <boost/thread/mutex.hpp>
#include "boost/numeric/ublas/matrix.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "nav_msgs/Path.h"
#include "underwater_slam/RequireControl.h"
#include "underwater_slam/Node.h"
#include "geometry_msgs/Point.h"
#include <pcl_ros/point_cloud.h>

struct Node 
{
	unsigned int seq;
	Eigen::Vector2d centre;
	std::vector<Eigen::Vector2d> size;
};

class Front_end
{
public:
	Front_end()
	{
		ready_pos_ = false;
		ready_q_ = false;
		ready_ = false;
		temp_cloud_.clear();
		temp_cloud_.header.frame_id = "world";
		pre_pos_ << 0, 0;
		count_num_ = 0;
		seq_ = 0;
	}

	void set_pos(Eigen::Vector2d& pos)
	{
		pos_ = pos;
		ready_pos_ = true;
	}

	void set_q(Eigen::Quaternionf& q)
	{
		q_ = q;
		ready_q_ = true;
	}

	pcl::PointCloud<pcl::PointXYZ>& points()
	{
		return cloud_;
	}

	bool ready()
	{
		return ready_;
	}

	void sented()
	{
		ready_ = false;
	}

	void lock()
	{
		mutex_.lock();
	}

	void unlock()
	{
		mutex_.unlock();
	}

	geometry_msgs::Point change_vec(Eigen::Vector2d vec)
	{
		geometry_msgs::Point point;
		point.x = vec[0];
		point.y = vec[1];
		point.z = 0;

		return point;
	}

	Eigen::Vector2d change_point(pcl::PointXYZ point)
	{
		Eigen::Vector2d vec;
		vec[0] = point.x;
		vec[1] = point.y;
		return vec;
	}

	underwater_slam::Node node_msg()
	{
		underwater_slam::Node message;
		message.seq = node_.seq;
		message.centre.x = node_.centre[0];
		message.centre.y = node_.centre[1];
		message.centre.z = 0;
		for (int i = 0; i < 4; ++i)
		{
			message.size.push_back(change_vec(node_.size[i]));
		}
		return message;
	}

	void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
		pcl::PointCloud<pcl::PointXYZ> cloud;
		if (ready_pos_&&ready_q_)
		{
			Eigen::Vector2d det = pos_ - pre_pos_;
			if (det.norm() < 0.05)
			{
				return;
			}
			boost::numeric::ublas::matrix<double> ranges(2, scan_in->ranges.size()-1);
  		boost::numeric::ublas::matrix<double> angle(2, scan_in->ranges.size()-1);

  		double z_max = 0;  
  		double angle_min = scan_in->angle_min;
  		double angle_max = scan_in->angle_max;
  		double angle_increment = scan_in->angle_increment;

  		for (unsigned int index = 0;index < scan_in->ranges.size()-1; index++)
  		{
    		angle(0,index) = cos(angle_min + (double) index * angle_increment);
    		angle(1,index) = sin(angle_min + (double) index * angle_increment);
  		}
  		for (unsigned int index = 0; index < scan_in->ranges.size()-1; index++)
  		{
		    ranges(0,index) = (double) scan_in->ranges[index];
		    ranges(1,index) = (double) scan_in->ranges[index];
		  }
		  boost::numeric::ublas::matrix<double> output = element_prod(ranges, angle);
		  for (unsigned int index = 0; index< scan_in->ranges.size()-1; index++)
		  {
		    pcl::PointXYZ point(0.0, output(1,index), -output(0,index)); 
		    cloud.push_back(point);
		  }
		  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		  transform.translation() << pos_(0), pos_(1), 0.0;
		  transform.rotate (q_);
		  pcl::transformPointCloud (cloud, cloud, transform);
		  temp_cloud_+=cloud;
		  if (count_num_ == 25)
		  {
		  	node_.centre = pos_;
		  }
		  if (count_num_ == 0 || count_num_ == 50)
		  {
		  	if (count_num_ == 0)
		  	{
		  		node_.size.clear();
		  	}
		  	node_.size.push_back(change_point(cloud[0]));
		  	node_.size.push_back(change_point(cloud[cloud.size()]));
		  }
		  if (count_num_ >= 50)
		  {
		  	cloud_ = temp_cloud_;
		  	cloud_.header.seq = seq_;
		  	node_.seq = seq_;
		  	seq_++;
		  	temp_cloud_.clear();
		  	mutex_.lock();
		  	ready_ = true;
		  	mutex_.unlock();
		  	count_num_ = -1;
		  }
		  count_num_++;
		  pre_pos_ = pos_;
		}
	}

private:
	Node node_;
	Eigen::Vector2d pos_;
	Eigen::Quaternionf q_;
	Eigen::Vector2d pre_pos_;
	pcl::PointCloud<pcl::PointXYZ> cloud_;
	pcl::PointCloud<pcl::PointXYZ> temp_cloud_;
	bool ready_pos_;
	bool ready_q_;
	bool ready_;
	int count_num_;
	unsigned int seq_;
	boost::mutex mutex_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "front_end");
	ros::NodeHandle node;

	Front_end front_end;
	underwater_slam::RequireControl control_srv;

	ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("g500/multibeam", 10, &Front_end::callback, &front_end);
  ros::ServiceClient control_client = node.serviceClient<underwater_slam::RequireControl>("request_control");
  ros::Publisher pub_path = node.advertise<nav_msgs::Path>("ImuPath", 1);
  ros::Publisher pub_points = node.advertise<pcl::PointCloud<pcl::PointXYZ>> ("Points", 1);
  ros::Publisher pub_node = node.advertise<underwater_slam::Node>("NodeMsg", 1);

	ros::Rate loop_rate(60);

	Eigen::Vector2d pos = Eigen::Vector2d::Zero();

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

		pos(0) = det(0)*cos(yaw) - det(1)*sin(yaw) + pos(0);
		pos(1) = det(0)*sin(yaw) + det(1)*cos(yaw) + pos(1);
		front_end.set_pos(pos);
		front_end.set_q(q);

		path_rst.header.stamp = ros::Time::now();
    path_rst.header.seq++;
    path_rst.header.frame_id = "/world";

    geometry_msgs::PoseStamped pose;
    
    pose.header = path_rst.header;
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = 0;
    pose.pose.orientation = control_srv.response.orientation;

    path_rst.poses.push_back(pose);
    pub_path.publish(path_rst);

    front_end.lock();
    if (front_end.ready())
    {
    	front_end.sented();
    	pub_points.publish(front_end.points());
    	pub_node.publish(front_end.node_msg());
    }
    front_end.unlock();

    ros::spinOnce();
  	loop_rate.sleep();
	}

	return 0;
}
