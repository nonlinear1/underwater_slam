#ifndef EKF_CORE_H
#define EKF_CORE_H

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

class EKF_core
{
public:
	EKF_core();
	EKF_core(double,double,double,double,double,double);
	~EKF_core(){};
	Eigen::VectorXd get_u()
	{
		return u_;
	}
	void set_sigma();
	void set_u();
	void set_R();
	void set_qt();
	bool process(Eigen::Vector3d control, std::vector<Eigen::Vector2d> points);
private:
	Eigen::MatrixXd sigma_;
	Eigen::VectorXd u_;
	Eigen::Matrix3d R_;
	Eigen::Matrix3d qt_;
	int N_;
};

#endif