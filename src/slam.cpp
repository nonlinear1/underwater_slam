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
#include "g2o/types_slam.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
 
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
    ros::Publisher pub_merge_points = node.advertise<pcl::PointCloud<pcl::PointXYZ>> ("Merge_Points", 1);

	ros::Rate loop_rate(60);

	Eigen::Vector2d pos = Eigen::Vector2d::Zero();
	Eigen::Vector2d start_pos = Eigen::Vector2d::Zero();

	ros::Time before, after;
  ros::Duration duration;
  double det_t;
  before = ros::Time::now();
  nav_msgs::Path path_rst;

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  GraphWithTree optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);
  optimizer.set_sigma_for_edge(0.1);
  optimizer.set_sigma_for_icp(0.1);

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
    	Node node = front_end.get_node();
    	front_end.sented();
    	pub_points.publish(node.cloud);

    	// add node to graph 
    	ROS_DEBUG_STREAM("POS: " << start_pos);
      	ROS_DEBUG_STREAM("Node" << front_end.get_node().seq << " sented");
    	optimizer.add_node(node);

      unsigned int times = 1;
      if (node.seq >= 5*times)
      {
        VertexPointXY* firstRobotPose = dynamic_cast<VertexPointXY*>(optimizer.vertex(0));
        firstRobotPose->setFixed(true);
        optimizer.setVerbose(true);

        ROS_DEBUG_STREAM("Optimizing...");
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        ROS_DEBUG_STREAM("Done");

        pub_merge_points.publish(optimizer.merge_cloud(node.seq));
        times++;
      }
    }
    front_end.unlock();

    ros::spinOnce();
  	loop_rate.sleep();
	}

	return 0;
}	