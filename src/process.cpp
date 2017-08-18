#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "underwater_slam/PointDetection.h"
#include "underwater_slam/RequireControl.h"

struct Feature
{
  bool fresh;
  Eigen::Vector3d z;
  Eigen::MatrixXd kti;
  Eigen::MatrixXd hti;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process");

  ros::NodeHandle node;
  ros::ServiceClient measurement_client = node.serviceClient<underwater_slam::PointDetection>("point_detection");
  ros::ServiceClient control_client = node.serviceClient<underwater_slam::RequireControl>("request_control");

  underwater_slam::PointDetection measurement_srv;
  underwater_slam::RequireControl control_srv;

  Eigen::MatrixXd sigma = Eigen::Matrix3d::Zero();
  Eigen::VectorXd u = Eigen::Vector3d::Zero();
  Eigen::MatrixXd fx;
  Eigen::Vector3d control; 
  Eigen::Vector3d new_m;
  Eigen::Matrix3d R;
  Eigen::Quaternion<double> q;
  Eigen::MatrixXd h_base = Eigen::MatrixXd::Zero(3,6);
  Eigen::Matrix3d qt = Eigen::Matrix3d::Identity()*0.01;
  Eigen::MatrixXd kti;
  int n = 0;

  h_base.block(0,0,3,3) = Eigen::Matrix3d::Identity() * (-1);
  h_base.block(0,3,3,3) = Eigen::Matrix3d::Identity();

  R = Eigen::Matrix3d::Identity();
  R *= 0.01;

  std::vector<Feature> feature_list;
  std::vector<double> dis_list;
  ros::Rate loop_rate(20);

  ros::Time before, after;
  ros::Duration duration;
  double det_t;
  before = ros::Time::now();
  ROS_INFO_STREAM("Begin count ..." << std::endl << "u:" << std::endl << u);

  while(ros::ok())
  {	
    ROS_INFO_STREAM("Load data ...");
    after = ros::Time::now();
    duration = after - before;
    before = after;
    det_t = duration.toSec();

    control_client.call(control_srv);
    measurement_client.call(measurement_srv);

    q = Eigen::Quaternion<double>(control_srv.response.orientation.w,
                                  control_srv.response.orientation.x,
                                  control_srv.response.orientation.y,
                                  control_srv.response.orientation.z);
    auto rotation = q.toRotationMatrix();
    double yaw = atan(rotation(1,0)/rotation(0,0)) - u(2);
    control << control_srv.response.linear_velocity.x*det_t, control_srv.response.linear_velocity.y*det_t, yaw;
    std::cout << "control: " << std::endl << control << std::endl; 
    ROS_INFO_STREAM("Update ...");

    Eigen::MatrixXd fx_temp = Eigen::MatrixXd::Zero(3,3+3*n);
    fx_temp.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    fx = fx_temp;

    u += fx.transpose()*control;

    sigma = sigma + fx.transpose()*R*fx;
    std::cout << "u: " << std::endl << u << std::endl;
    std::cout << "sigma: " << std::endl << sigma << std::endl;
    ROS_INFO_STREAM("Measurement ...");

    for(int i = 0; i < measurement_srv.response.res.points.size(); i++)
    { 
      ROS_INFO_STREAM( i+1 << "th measurement");
      new_m(0) = measurement_srv.response.res.points[i].x + u(0);
      new_m(1) = measurement_srv.response.res.points[i].y + u(1);
      new_m(2) = 0;
      std::cout << "new_m: " << std::endl << new_m << std::endl;

      for(int j = 0; j < n; j++)
      {
        Eigen::Vector3d temp;
        temp(0) = u(3+j*3);
        temp(1) = u(4+j*3);
        temp(2) = 0;
        
        Eigen::MatrixXd fxk = Eigen::MatrixXd::Zero(6,3+3*n);
        fxk.block(0,0,3,3) = Eigen::Matrix3d::Identity();
        fxk.block(3,3+3*j,3,3) = Eigen::Matrix3d::Identity();
        
        Eigen::MatrixXd htk;
        htk = h_base*fxk;
        
        Eigen::MatrixXd psi_k;
        psi_k = htk*sigma*htk.transpose()+qt;
        std::cout << "new_m-temp: " << std::endl << new_m-temp << std::endl;
        double dis = (new_m-temp).transpose() * psi_k.inverse()*(new_m-temp);
        if (dis < 0)
        {
          dis = -dis;
        }
        dis_list.push_back(dis);
        ROS_INFO_STREAM("Push dis " << dis);
      }

      int min_ind = n+1;
      double min = 5; 

      for(int k = 0; k < dis_list.size(); k++)
      {
        if(dis_list[k] < min)
        {
          min = dis_list[k];
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
        std::cout << "pos: " << std::endl << feature.z << std::endl;
        feature.kti = qt;
        feature_list.push_back(feature);
      }
      else 
      {
        ROS_INFO_STREAM("Update old");

        feature.fresh = false;
        feature.z = new_m - Eigen::Vector3d(u(3+min_ind*3), u(4+min_ind*3), 0);

        Eigen::MatrixXd fxk = Eigen::MatrixXd::Zero(6,3+3*n);
        fxk.block(0,0,3,3) = Eigen::Matrix3d::Identity();
        fxk.block(3,3+3*min_ind,3,3) = Eigen::Matrix3d::Identity();
        feature.hti = h_base*fxk;
        Eigen::MatrixXd psi_k;
        psi_k = feature.hti*sigma*feature.hti.transpose()+qt;
        
        feature.kti = sigma * feature.hti.transpose() * psi_k.inverse();
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
        psi_temp += (*it).kti*(*it).hti;
      }
    }
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
    feature_list.clear();

  	ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}