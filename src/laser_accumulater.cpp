#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_listener.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "boost/numeric/ublas/matrix.hpp"
#include <vector>
#include "accumulater.h"

#define num_of_split 5

int main(int argc, char **argv)
{
  std::vector<sensor_msgs::PointCloud> cloud_rst;
  cloud_rst.resize(num_of_split);

  ros::init(argc, argv, "laser_convey");
  tf::StampedTransform transform;
  tf::TransformListener listener;
  geometry_msgs::Transform pos;
  Accumulater accumulater(num_of_split);

  ros::NodeHandle node;
  
  ros::Publisher chatter_pub = node.advertise<sensor_msgs::PointCloud>("PointCloudSonar", 1);

  ros::Publisher transform_pub = node.advertise<geometry_msgs::Transform>("PointCloudTransform", 1);
  
  ros::Subscriber sub = node.subscribe("g500/multibeam", 10, &Accumulater::callback, &accumulater);

  ros::Rate loop_rate(1000);
  
  int count = 0;
  bool first = true;

  while (ros::ok())
  {
    if (count == 0 && first)
    {
      accumulater.lock();
      accumulater.init();
      accumulater.unlock();
    }
    if (accumulater.ifready())
    {
      accumulater.copy_pointcloud(cloud_rst);
      count++;
    }
    if (count > 0 && count % 10 == 0 && first)
    {
      accumulater.lock();
      if (count != 50)
      {
        cloud_rst[count/10].points.clear();
        accumulater.unlock();
      }
      else 
      {
        first = false;
        accumulater.copy_transform(pos);
        transform_pub.publish(pos);
        chatter_pub.publish(cloud_rst[0]);
        cloud_rst[0].points.clear();
        cloud_rst.push_back(cloud_rst[0]);
        cloud_rst.erase(cloud_rst.begin());
        accumulater.setbase();
        count = 0;
      }
      accumulater.unlock();
    }
    else if (count == 10 && !first)
    {
      accumulater.lock();
      chatter_pub.publish(cloud_rst[0]);
      cloud_rst[0].points.clear();
      cloud_rst.push_back(cloud_rst[0]);
      cloud_rst.erase(cloud_rst.begin());
      accumulater.setbase();
      count = 0;
      accumulater.unlock();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}