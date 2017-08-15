#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_listener.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "boost/numeric/ublas/matrix.hpp"

sensor_msgs::PointCloud cloud_;
boost::mutex mutex;
bool ready = false;

void transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform)
{
  boost::numeric::ublas::matrix<double> ranges(2, scan_in.ranges.size()-1);
  boost::numeric::ublas::matrix<double> angle(2, scan_in.ranges.size()-1);
  cloud_out.points.resize (scan_in.ranges.size()-1);
  
  double angle_min = scan_in.angle_min;
  double angle_max = scan_in.angle_max;
  double angle_increment = scan_in.angle_increment;

  for (unsigned int index = 0;index < scan_in.ranges.size()-1; index++)
  {
    angle(0,index) = cos(angle_min + (double) index * angle_increment);
    angle(1,index) = sin(angle_min + (double) index * angle_increment);
  }
  for (unsigned int index = 0; index < scan_in.ranges.size()-1; index++)
  {
    ranges(0,index) = (double) scan_in.ranges[index];
    ranges(1,index) = (double) scan_in.ranges[index];
  }
  boost::numeric::ublas::matrix<double> output = element_prod(ranges, angle);
  for (unsigned int index = 0; index< scan_in.ranges.size()-1; index++)
  {
    tf::Vector3 point_in(0.0, output(1,index), -output(0,index)); 
    tf::Vector3 point_out = transform * point_in;
    cloud_out.points[index].x = point_out.getX();
    cloud_out.points[index].y = point_out.getY();
    cloud_out.points[index].z = point_out.getZ();
  }
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/girona500", "/world",
                               ros::Time(0), ros::Duration(1.0));
  listener.lookupTransform("/girona500", "/world",
                             ros::Time(0), transform);
  mutex.lock();
  transLaser(*scan_in, cloud_, transform);
  cloud_.header = scan_in->header;
  cloud_.header.frame_id = "/world";
  ready = true;
  mutex.unlock();
}

int main(int argc, char **argv)
{
  sensor_msgs::PointCloud cloud_rst1;
  sensor_msgs::PointCloud cloud_rst2;

  ros::init(argc, argv, "laser_convey");

  ros::NodeHandle n_sub;
  ros::NodeHandle n_adv;
  
  ros::Publisher chatter_pub = n_adv.advertise<sensor_msgs::PointCloud>("PointCloudConvey", 1);
  
  ros::Subscriber sub = n_sub.subscribe("g500/multibeam", 1, scanCallback);

  ros::Rate loop_rate(1000);
  
  int count = 0;
  bool first = true;
  while (ros::ok())
  {
    if (ready)
    {
      mutex.lock();
      cloud_rst1.header = cloud_.header;
      cloud_rst2.header = cloud_.header;
      for (unsigned int index = 0; index< cloud_.points.size(); index++)
      {
        cloud_rst1.points.push_back(cloud_.points[index]);
        cloud_rst2.points.push_back(cloud_.points[index]);
      }
      ready = false;
      mutex.unlock();
      count++;
    }
    if (count == 10)
    {
      chatter_pub.publish(cloud_rst1);
      count = 0;
      cloud_rst1.points.clear();
    }

    if (count == 5 && !first)
    {
      chatter_pub.publish(cloud_rst2);
      cloud_rst2.points.clear();
    }
    else if (count == 5 && first)
    {
      first = false;
      cloud_rst2.points.clear();
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}