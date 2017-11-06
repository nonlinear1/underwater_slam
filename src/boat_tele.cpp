#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>


class TeleopUWsim
{
public:
  TeleopUWsim();
  void publish();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  boost::mutex mutex_;
  nav_msgs::Odometry msg_;
};


TeleopUWsim::TeleopUWsim():
  linear_(1),
  angular_(3)
{
  l_scale_ = a_scale_ = 1;
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<nav_msgs::Odometry>("/dataNavigator", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopUWsim::joyCallback, this);

}

void TeleopUWsim::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  mutex_.lock();
  msg_.twist.twist.angular.z = a_scale_*joy->axes[angular_];
  msg_.twist.twist.linear.x = l_scale_*joy->axes[linear_];
  mutex_.unlock();
}

void TeleopUWsim::publish()
{
  mutex_.lock();
  vel_pub_.publish(msg_);
  mutex_.unlock();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopUWsim teleop_turtle;

  ros::Rate loop_rate(100);
  
  while(ros::ok())
  { 
    teleop_turtle.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}