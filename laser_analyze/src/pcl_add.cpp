#include "ros/ros.h"
#include <boost/thread.hpp>
#include "sensor_msgs/PointCloud.h"

sensor_msgs::PointCloud cloud;
bool ready = false;
boost::mutex mutex;

void scanCallback (const sensor_msgs::PointCloud::ConstPtr& point_in)
{
  mutex.lock();
  cloud.header = point_in->header;
  cloud.header.frame_id = "/world";
  for (unsigned int index = 0; index< point_in->points.size(); index++)
  {
  	cloud.points.push_back(point_in->points[index]);
  }
  ready = true;
  mutex.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_add");

  ros::NodeHandle n_sub;
  ros::NodeHandle n_adv;
  
  ros::Publisher chatter_pub = n_adv.advertise<sensor_msgs::PointCloud>("PointCloudAdd", 1);
  
  ros::Subscriber sub = n_sub.subscribe("PointCloudConvey", 1, scanCallback);

  ros::Rate loop_rate(1000);
  
  while (ros::ok())
  {
  	mutex.lock();
    if (ready)
    {
      chatter_pub.publish(cloud);
    }
    mutex.unlock();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}