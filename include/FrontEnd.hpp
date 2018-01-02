#ifndef FRONTEND_HPP
#define FRONTEND_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>
#include "sensor_msgs/LaserScan.h"
#include <boost/thread/mutex.hpp>
#include "boost/numeric/ublas/matrix.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include "Node.hpp"

class FrontEnd
{
public:
	FrontEnd()
	{
		ready_pos_ = false;
		ready_q_ = false;
		ready_ = false;
		first_ = true;
		pre_pos_ << 0, 0;
		det_pos_ << 0, 0;
		pos_ << 0, 0;
		cloud_.clear();
		count_num_ = 0;
		seq_ = 0;
	}

	void set_pos(Eigen::Vector2d& pos)
	{
		det_pos_ += pos;
		ready_pos_ = true;
	}

	void set_q(Eigen::Quaternionf& q)
	{
		q_ = q;
		ready_q_ = true;
	}

	void lock()
	{
		mutex_.lock();
	}

	void unlock()
	{
		mutex_.unlock();
	}

	bool ready()
	{
		return ready_;
	}

	void sented()
	{
		ready_ = false;
	}

	void check_bound(pcl::PointXYZ& point)
	{
		if (first_)
		{
			node_.bound[0] = point.x;
			node_.bound[1] = point.x;
			node_.bound[2] = point.y;
			node_.bound[3] = point.y;
			first_ = false;
			return;
		}

		if (point.x < node_.bound[0])
		{
			node_.bound[0] = point.x;
		}
		else if (point.x > node_.bound[1])
		{
			node_.bound[1] = point.x;
		}

		if (point.y < node_.bound[2])
		{
			node_.bound[2] = point.x;
		}
		else if (point.y > node_.bound[3])
		{
			node_.bound[3] = point.y;
		}
	}

	Node get_node()
	{
		return node_;
	}

	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{	
		pcl::PointCloud<pcl::PointXYZ> laser;

		if (ready_pos_&&ready_q_)
		{			
			if (det_pos_.norm() < 0.05)
			{
				return;
			}

			pos_ += det_pos_;
			det_pos_ = Eigen::Vector2d::Zero();
			
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
		    laser.push_back(point);
		  }
		  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		  transform.translation() << pos_(0), pos_(1), 0.0;
		  transform.rotate (q_);
		  pcl::transformPointCloud (laser, laser, transform);

		  check_bound(laser[0]);
		  check_bound(laser[laser.size()-1]);

		  cloud_+=laser;
		  
		  if (count_num_ == 25)
		  {
		  	node_.centre = pos_;
		  	node_.centre_q = q_;
		  }
		  
		  if (count_num_ >= 50)
		  {
		  	node_.seq = seq_;
		  	transform = Eigen::Affine3f::Identity();
		  	transform.translation() << -node_.centre(0), -node_.centre(1), 0.0;
		  	pcl::transformPointCloud (cloud_, node_.cloud, transform);
		  	node_.cloud.header.frame_id = "world";
		  	Eigen::Vector2d temp_pos = node_.centre + pre_pos_;
		  	pre_pos_ << pos_(0)-node_.centre(0), pos_(1)-node_.centre(1);
		  	node_.centre = temp_pos;
		  	
		  	ROS_DEBUG_STREAM("Node" << seq_ << " finished!");
		  	ROS_DEBUG_STREAM("Node" << seq_ << " centre: " << node_.centre);

		  	seq_++;
		  	cloud_.clear();
		  	mutex_.lock();
		  	ready_ = true;
		  	mutex_.unlock();
		  	count_num_ = -1;
		  	pos_ = Eigen::Vector2d::Zero();
		  	first_ = true;
		  }
		  count_num_++;
		}
	}

private:
	Node node_;
	pcl::PointCloud<pcl::PointXYZ> cloud_;
	Eigen::Vector2d pos_;
	Eigen::Vector2d det_pos_;
	Eigen::Vector2d pre_pos_;
	Eigen::Quaternionf q_;
	bool ready_pos_;
	bool ready_q_;
	bool ready_;
	bool first_;
	int count_num_;
	unsigned int seq_;
	boost::mutex mutex_;
};

#endif