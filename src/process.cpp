#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"
#include "underwater_slam/PointDetection.h"
#include "underwater_slam/RequireControl.h"
#include "EKF_core.h"

struct Feature
{
  bool fresh;
  Eigen::Vector3d z;
  Eigen::MatrixXd kti;
  Eigen::MatrixXd hti;
};

struct Dis_item
{ 
  double dis;
  Eigen::Vector3d det;
  Eigen::MatrixXd h;
  Eigen::MatrixXd psi;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process");

  ros::NodeHandle node;
  ros::ServiceClient measurement_client = node.serviceClient<underwater_slam::PointDetection>("point_detection");
  ros::ServiceClient control_client = node.serviceClient<underwater_slam::RequireControl>("request_control");

  ros::Publisher pub_point = node.advertise<sensor_msgs::PointCloud>("Keypoint", 1);
  ros::Publisher pub_path = node.advertise<nav_msgs::Path>("Boatpath", 1);

  underwater_slam::PointDetection measurement_srv;
  underwater_slam::RequireControl control_srv;

  sensor_msgs::PointCloud point_rst;
  nav_msgs::Path path_rst;

  EKF_core core(0.01,0.01,0.0001,0.1,0.1,0.1);

  ros::Rate loop_rate(10);

  ros::Time before, after;
  ros::Duration duration;
  double det_t;
  before = ros::Time::now();
  std::vector<Eigen::Vector2d> points;

  while(ros::ok())
  {	
    after = ros::Time::now();
    duration = after - before;
    before = after;
    det_t = duration.toSec();

    if(!control_client.call(control_srv))
    {
      ROS_ERROR_STREAM("control serve fail!");
    }
    if(!measurement_client.call(measurement_srv))
    {
      ROS_ERROR_STREAM("measurement serve fail!");
    }

    double yaw = control_srv.response.yaw;
    
    Eigen::Vector3d control, control_new;
    control << control_srv.response.linear_velocity.x*det_t, control_srv.response.linear_velocity.y*det_t, yaw;
    
    control_new(0) = control(0)*cos(yaw) - control(1) * sin(yaw);
    control_new(1) = control(1)*cos(yaw) + control(0) * sin(yaw);
    control_new(2) = control(2);

    points.clear();
    for(int i = 0; i < measurement_srv.response.res.points.size(); i++)
    {
      Eigen::Vector2d point(measurement_srv.response.res.points[i].x, measurement_srv.response.res.points[i].y);
      points.push_back(point);
    }
    
    core.process(control_new, points);

    Eigen::VectorXd u = core.get_u();

    point_rst.header.stamp = ros::Time::now();
    point_rst.header.seq++;
    point_rst.header.frame_id = "/world";

    geometry_msgs::Point32 point;

    for (int i = 1; i < (u.rows()/3); ++i)
    {
      point.x = u(3*i);
      point.y = u(3*i+1);
      point.z = 0;
      point_rst.points.push_back(point);
    }

    pub_point.publish(point_rst);
    point_rst.points.clear(); 

    path_rst.header.stamp = ros::Time::now();
    path_rst.header.seq++;
    path_rst.header.frame_id = "/world";

    geometry_msgs::PoseStamped pose;
    pose.header = path_rst.header;
    pose.pose.position.x = u(0);
    pose.pose.position.y = u(1);
    pose.pose.position.z = 0;
    pose.pose.orientation = control_srv.response.orientation;

    path_rst.poses.push_back(pose);

    pub_path.publish(path_rst);
    
  	ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}