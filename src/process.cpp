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

#define pie 3.141592653589793 

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

  Eigen::MatrixXd sigma = Eigen::Matrix3d::Zero();
  Eigen::VectorXd u = Eigen::Vector3d::Zero();
  Eigen::MatrixXd fx;
  Eigen::Vector3d control; 
  Eigen::Vector3d new_m;
  Eigen::Matrix3d R;
  Eigen::Quaternion<double> q;
  Eigen::Matrix3d qt = Eigen::Matrix3d::Identity();
  qt *= 0.1;
  Eigen::MatrixXd kti;
  int n = 0;

  R = Eigen::Matrix3d::Identity();
  R *= 0.01;
  R(2,2) = 0.0001;

  std::vector<Feature> feature_list;
  std::vector<Dis_item> dis_list;
  ros::Rate loop_rate(10);

  ros::Time before, after;
  ros::Duration duration;
  double det_t;
  before = ros::Time::now();

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

    double yaw = tf::getYaw(control_srv.response.orientation);
    
    control << control_srv.response.linear_velocity.x*det_t, control_srv.response.linear_velocity.y*det_t, yaw-u(2);

    Eigen::MatrixXd fx_temp = Eigen::MatrixXd::Zero(3,3+3*n);
    fx_temp.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    fx = fx_temp;

    Eigen::Vector3d control_new;
    control_new(0) = control(0)*cos(yaw) - control(1) * sin(yaw);
    control_new(1) = control(1)*cos(yaw) + control(0) * sin(yaw);
    control_new(2) = control(2);

    std::cout << "control: " << std::endl << control_new << std::endl; 
    
    u += fx.transpose()*control_new;

    sigma = sigma + fx.transpose()*R*fx;
    
    std::cout << "u: " << std::endl << u << std::endl;
    std::cout << "sigma: " << std::endl << sigma << std::endl;
    
    for(int i = 0; i < measurement_srv.response.res.points.size(); i++)
    { 
      ROS_INFO_STREAM( i+1 << "th measurement");
      double x, y;
      x = measurement_srv.response.res.points[i].x;
      y = measurement_srv.response.res.points[i].y;
      Eigen::Vector3d z(x,y,0);
      new_m(0) = x*cos(yaw) - y*sin(yaw) + u(0);
      new_m(1) = x*sin(yaw) + y*cos(yaw) + u(1);
      new_m(2) = 0;
      std::cout << "new_m: " << std::endl << new_m << std::endl;
      std::cout << "z: " << std::endl << z << std::endl;

      for(int j = 0; j < n; j++)
      {
        Eigen::Vector3d det_v;
        det_v(0) = u(3+j*3) - u(0);
        det_v(1) = u(4+j*3) - u(1);
        det_v(2) = 0;

        Eigen::Vector3d zkt;
        zkt(0) = det_v(0) * cos(yaw) + det_v(1) * sin(yaw);
        zkt(1) = (-1) * det_v(0) * sin(yaw) + det_v(1) * cos(yaw);
        zkt(2) = 0;
        std::cout << "zkt: " << std::endl << zkt << std::endl;
        
        Eigen::MatrixXd fxk = Eigen::MatrixXd::Zero(6,3+3*n);
        fxk.block(0,0,3,3) = Eigen::Matrix3d::Identity();
        fxk.block(3,3+3*j,3,3) = Eigen::Matrix3d::Identity();
        
        Eigen::MatrixXd h_base(3,6);
        h_base << -cos(yaw),  -sin(yaw), zkt(1),  cos(u(2)),  sin(u(2)), 0,
                  sin(yaw),   -cos(yaw), -zkt(0), -sin(u(2)), cos(u(2)), 0,
                  0,           0,          0,       0,          0,         1; 
    
        Eigen::MatrixXd htk;
        htk = h_base*fxk;
        std::cout << "htk: " << std::endl << htk << std::endl;
        
        Eigen::MatrixXd psi_k;
        psi_k = htk*sigma*htk.transpose()+qt;
        std::cout << "z-zkt: " << std::endl << z-zkt << std::endl;
        std::cout << "psi_k: " << std::endl << psi_k << std::endl;
        double dis = (z-zkt).transpose() * psi_k.inverse()*(z-zkt);
        if (std::isnan(dis))
        {
          ROS_ERROR_STREAM("NAN");
        }
        if (dis < 0)
        {
          dis = -dis;
        }
        Dis_item item;
        item.dis = dis;
        item.det = z-zkt;
        item.h = htk;
        item.psi = psi_k;
        dis_list.push_back(item);
        ROS_INFO_STREAM("Push dis " << dis);
      }

      int min_ind = n+1;
      double min = 10; 

      for(int k = 0; k < dis_list.size(); k++)
      {
        if(dis_list[k].dis < min)
        {
          min = dis_list[k].dis;
          min_ind = k;
        }
      }
      
      Feature feature;
      
      if (min_ind == n+1)
      {
        ROS_INFO_STREAM("New find");
        kti = Eigen::Matrix3d::Zero();
        feature.fresh = true;
        feature.z = new_m;
        feature.kti = qt;
        feature_list.push_back(feature);
      }
      else 
      {
        ROS_INFO_STREAM("Update old");

        feature.fresh = false;
        feature.z = dis_list[min_ind].det;
        feature.hti = dis_list[min_ind].h;        
        feature.kti = sigma * feature.hti.transpose() * dis_list[min_ind].psi.inverse();
        feature_list.push_back(feature);
      }
      dis_list.clear();
    } 
    
    Eigen::VectorXd u_temp = Eigen::VectorXd::Zero(3+3*n);
    Eigen::MatrixXd psi_temp = Eigen::MatrixXd::Zero(3+3*n,3+3*n);
    
    std::vector<Feature>::iterator it;
    for (it = feature_list.begin(); it != feature_list.end(); ++it)
    {      
      if (!(*it).fresh)
      {
        
        u_temp += (*it).kti*(*it).z;
        std::cout << "detz: " << std::endl << (*it).z << std::endl;
        std::cout << "kti: " << std::endl << (*it).kti << std::endl;
        psi_temp += (*it).kti*(*it).hti;
      }
    }
    std::cout << "u_temp: " << std::endl << u_temp << std::endl;
    std::cout << "psi_temp: " << std::endl << psi_temp << std::endl;
    u += u_temp;
    sigma = (Eigen::MatrixXd::Identity(3+3*n,3+3*n) - psi_temp) * sigma;
    
    for (it = feature_list.begin(); it != feature_list.end(); ++it)
    {
      if ((*it).fresh)
      {
        n=n+1;
        Eigen::VectorXd u_temp(3+3*n);
        u_temp.head(3*n) = u;
        u_temp(3*n) = (*it).z(0);
        u_temp(3*n+1) = (*it).z(1);
        u_temp(3*n+2) = 0;
        u = u_temp;
        Eigen::MatrixXd sigma_temp = Eigen::MatrixXd::Zero(3+3*n,3+3*n);
        sigma_temp.block(0,0,3*n,3*n) = sigma;
        sigma_temp.block(3*n,3*n,3,3) = (*it).kti;
        sigma = sigma_temp;       
      }
    }
    std::cout << "u: " << std::endl << u << std::endl;
    std::cout << "next" << std::endl;
    feature_list.clear();

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