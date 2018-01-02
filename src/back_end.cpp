#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include "underwater_slam/Node.h"

struct Node 
{
	Eigen::Vector2d centre;
	std::vector<Eigen::Vector2d> size;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	unsigned int seq;
	Node(Eigen::Vector2d centre_in, std::vector<Eigen::Vector2d> size_in, unsigned int seq_in)
	{
		centre = centre_in;
		size = size_in;
		seq = seq_in;
	}
	Node(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
	{
		cloud = cloud_in;
	}
};


class BackEnd
{
public:
	BackEnd()
	{
		offset_ = 0;
		data_.clear();
		first_ = true;
	}
	~BackEnd() {}

	void node_callback(const underwater_slam::Node::ConstPtr& node)
	{
		if(first_)
		{
			offset_ = node->seq;
			first_ = false;
		}
		mutex_.lock();
		if (data_.size() > node->seq - offset_)
		{
			data_[node->seq-offset_].centre = trans_point(node->centre);
			data_[node->seq-offset_].size = trans_size(node->size);
			data_[node->seq-offset_].seq = node->seq - offset_;
		}
		else
		{
			Node* node_new = new Node(trans_point(node->centre), trans_size(node->size), node->seq - offset_);
			data_.push_back(*node_new);
			std::cout  << "add node" << node->seq << std::endl;

		}
		mutex_.unlock();
	}

	void cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
	{
		if(first_)
		{
			offset_ = cloud->header.seq;
			first_ = false;
		}
		mutex_.lock();
		if (data_.size() > cloud->header.seq - offset_)
		{
			data_[cloud->header.seq-offset_].cloud = cloud->makeShared();
		}
		else
		{
			Node* node_new = new Node(cloud->makeShared());
			data_.push_back(*node_new);
			std::cout  << "add node" << cloud->header.seq << std::endl;
		}
		mutex_.unlock();
	}

private:

	Eigen::Vector2d trans_point(geometry_msgs::Point point)
	{
		Eigen::Vector2d vec;
		vec << point.x, point.y;

		return vec;
	}

	std::vector<Eigen::Vector2d> trans_size(std::vector<geometry_msgs::Point> points)
	{
		std::vector<Eigen::Vector2d> size;
		for(int i = 0; i < points.size(); i++)
		{
			size.push_back(trans_point(points[i]));
		}

		return size;
	}

	std::vector<Node> data_;
	boost::mutex mutex_;
	unsigned int offset_;
	bool first_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "back_end");
	ros::NodeHandle node;

	BackEnd back_end;

	ros::Subscriber sub_node = node.subscribe<underwater_slam::Node>("NodeMsg", 10, &BackEnd::node_callback, &back_end);
	ros::Subscriber sub_point = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("Points", 10, &BackEnd::cloud_callback, &back_end);

	ros::spin();

	return 0;
}	