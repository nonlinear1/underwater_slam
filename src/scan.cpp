#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_listener.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "boost/numeric/ublas/matrix.hpp"
#include <vector>

sensor_msgs::PointCloud cloud_sum;
bool ready;
boost::mutex mutex;

void transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform)
{
  boost::numeric::ublas::matrix<double> ranges(2, scan_in.ranges.size()-1);
  boost::numeric::ublas::matrix<double> angle(2, scan_in.ranges.size()-1);
  cloud_out.points.resize (scan_in.ranges.size()-1);

  double z_max = 0;  
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

void callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform("/world", "/girona500",
                               ros::Time(0), ros::Duration(1.0));
  listener.lookupTransform("/world", "/girona500",
                             ros::Time(0), transform);

  cloud.header = scan_in->header;
  cloud.header.frame_id = "/world";
  transLaser(*scan_in, cloud, transform);
  mutex.lock();
  cloud_sum.header = cloud.header;
  cloud_sum.points.insert(cloud_sum.points.end(), cloud.points.begin(), cloud.points.end());
  mutex.unlock();
  ready = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan");

	ready = false;

  ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("g500/multibeam", 10, callback);
	ros::Publisher chatter_pub = node.advertise<sensor_msgs::PointCloud>("PointCloudScan", 1);

	ros::Rate loop_rate(1000);

	while(ros::ok())
	{
		if (ready)
		{
			mutex.lock();
			chatter_pub.publish(cloud_sum);
			mutex.unlock();
			ready = false;
		}
		ros::spinOnce();
    loop_rate.sleep();
  }

}