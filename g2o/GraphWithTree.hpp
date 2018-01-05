#ifndef GRAPHWITHTREE_HPP
#define GRAPHWITHTREE_HPP

#include "vertex_point.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include "Node.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry> 

class GraphWithTree : public g2o::SparseOptimizer 
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	GraphWithTree(){}
	~GraphWithTree(){}

	void set_sigma_for_edge(double s)
	{
		sigma_for_edge_ = s;
	}

	void set_sigma_for_icp(double s)
	{
		sigma_for_icp_ = s;
	}

	bool add_node(Node node){
		unsigned int num = node.seq;
		VertexPointXY* vertex_xy = new VertexPointXY;
		vertex_xy->setId(num);
		Eigen::Vector2d pos;
		if (num == 0)
		{
			pos = node.centre;
		}
		else 
		{
			VertexPointXY* vec = static_cast<VertexPointXY*>(vertex(num-1));
			pos = *vec + node.centre ;
		}
		vertex_xy->setEstimate(pos);
		vertex_xy->set_r(node.R);
		vertex_xy->set_cloud(node.cloud);
		if (!addVertex(vertex_xy)){
		 	return false;
		} 
		else 	ROS_DEBUG_STREAM("Add node" << num << ": (" << pos(0) << ", " << pos(1) << ")");

		if (num == 0)
		{
			return true;
		}

		Eigen::Matrix<double, 2, 2, Eigen::ColMajor> sigma_edge =  Eigen::Matrix<double, 2, 2, Eigen::ColMajor>::Identity() * sigma_for_edge_;
		EdgePointXY* edge_pointxy = new EdgePointXY;
		edge_pointxy->vertices()[0] = vertex(num-1);
		edge_pointxy->vertices()[1] = vertex(num);
		edge_pointxy->setMeasurement(node.centre);
    edge_pointxy->setInformation(sigma_edge);
		if (!addEdge(edge_pointxy))
		{
			return false;
		}
		else 	ROS_DEBUG_STREAM("Add edge" << ": (" << node.centre(0) << ", " << node.centre(1) << ") from " << num-1 << " to " << num);

		check_points(num);

		return true;
	}

	void check_points(unsigned int num){
		for (unsigned int i = 0; i < num; i++)
		{
			if (distence(i, num) < 0)
			{
				ROS_DEBUG_STREAM("find " << i << " and " << num);
				icp_cloud(i, num);
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZ> merge_cloud(unsigned int n)
	{
		pcl::PointCloud<pcl::PointXYZ> source;
		pcl::PointCloud<pcl::PointXYZ> rst;
		for(unsigned int i = 0; i <= n; i++)
		{
			pcl::PointCloud<pcl::PointXYZ> transform_cloud;
			VertexPointXY* vec = static_cast<VertexPointXY*>(vertex(i));
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.translation() << vec->estimate()(0), vec->estimate()(1), 0.0;
			pcl::transformPointCloud (vec->cloud(), transform_cloud, transform);

			source += transform_cloud;
		}

		pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (source.makeShared());
  	sor.setLeafSize (0.1f, 0.1f, 0.1f);
  	sor.filter (rst);

  	rst.header.frame_id = "world";

  	return rst;
	}

private:

	bool icp_cloud(unsigned int n1, unsigned int n2){
		VertexPointXY* vec1 = static_cast<VertexPointXY*>(vertex(n1));
 		VertexPointXY* vec2 = static_cast<VertexPointXY*>(vertex(n2));
 		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
 		icp.setInputSource(vec1->cloud().makeShared());
  	icp.setInputTarget(vec2->cloud().makeShared());

  	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (0.05);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (50);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (1);

		Eigen::Vector2d det = vec2->estimate() - vec1->estimate();
		ROS_DEBUG_STREAM(det);

  	pcl::PointCloud<pcl::PointXYZ> Final;
  	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << det(0), det(1), 0.0;

  	icp.align(Final, transform.matrix());
  	ROS_DEBUG_STREAM("has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore());

  	Eigen::Matrix4f matrix = icp.getFinalTransformation();
  	Eigen::Vector2d det_icp;
  	det_icp << matrix(0,3), matrix(1,3);

		Eigen::Matrix<double, 2, 2, Eigen::ColMajor> sigma_icp =  Eigen::Matrix<double, 2, 2, Eigen::ColMajor>::Identity() * sigma_for_icp_;
		EdgePointXY* edge_pointxy = new EdgePointXY;
		edge_pointxy->vertices()[0] = vertex(n1);
		edge_pointxy->vertices()[1] = vertex(n2);
		edge_pointxy->setMeasurement(det_icp);
    edge_pointxy->setInformation(sigma_icp);
		if (!addEdge(edge_pointxy))
		{
			return false;
		}
		else 	ROS_DEBUG_STREAM("Add edge" << ": (" << det_icp(0) << ", " << det_icp(1) << ") from " << n1 << " to " << n2);
  	
  	return true;
	}

	double distence(unsigned int n1, unsigned int n2){
		VertexPointXY* vec1 = static_cast<VertexPointXY*>(vertex(n1));
 		VertexPointXY* vec2 = static_cast<VertexPointXY*>(vertex(n2));
 		return vec1->distence_to(*vec2) - 0.5*vec1->r() - 0.5*vec2->r();	
 	}

	double sigma_for_edge_;
	double sigma_for_icp_;
};

#endif