#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_listener.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "boost/numeric/ublas/matrix.hpp"
#include <vector>

#define num_of_split 5
#define line_in_frame 50


class Convery
{
public:
  Convery(){
    transform_base_list_.resize(num_of_split);
    cloud_list_.resize(num_of_split);
    ready_ = false;
  }
  ~Convery(){}
  
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void lock(){mutex_.lock();}
  void unlock(){mutex_.unlock();}
  void ready(){ready_ = true;}
  void clear(){ready_ = false;}
  bool ifready(){return ready_;}
  void setbase();
  void init();
  void transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform);

  std::vector<sensor_msgs::PointCloud> cloud_list_;

private:
  std::vector<tf::StampedTransform> transform_base_list_;
  tf::TransformListener listener_;
  boost::mutex mutex_;
  bool ready_;
};

void Convery::init()
{
  tf::StampedTransform transform;
  listener_.waitForTransform("/world", "/girona500",
                               ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform("/world", "/girona500",
                             ros::Time(0), transform);
  transform_base_list_.resize(num_of_split, transform);
}

void Convery::setbase()
{
  tf::StampedTransform transform;
  listener_.waitForTransform("/world", "/girona500",
                               ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform("/world", "/girona500",
                             ros::Time(0), transform);  
  transform_base_list_.erase(transform_base_list_.begin());
  transform_base_list_.push_back(transform);
}

void Convery::callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  tf::StampedTransform transform;
  tf::StampedTransform transform_temp;
  listener_.waitForTransform("/girona500", "/world",
                               ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform("/girona500", "/world",
                             ros::Time(0), transform);

  cloud.header = scan_in->header;
  cloud.header.frame_id = "/world";
  lock();
  for (int i = 0; i < num_of_split; ++i)
  {
    transform_temp = transform;
    transform *= transform_base_list_[i];
    transLaser(*scan_in, cloud, transform);
    cloud_list_[i] = cloud;
    transform = transform_temp;
  }
  ready();
  unlock();
}

void Convery::transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform)
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

int main(int argc, char **argv)
{
  std::vector<sensor_msgs::PointCloud> cloud_rst;
  cloud_rst.resize(num_of_split);

  ros::init(argc, argv, "laser_convey");
  tf::StampedTransform transform;
  tf::TransformListener listener;
  Convery convery;

  ros::NodeHandle n_sub;
  ros::NodeHandle n_adv;
  
  ros::Publisher chatter_pub = n_adv.advertise<sensor_msgs::PointCloud>("PointCloudConveyNow", 1);
  
  ros::Subscriber sub = n_sub.subscribe("g500/multibeam", 10, &Convery::callback, &convery);

  ros::Rate loop_rate(1000);
  
  int count = 0;
  bool first = true;

  while (ros::ok())
  {
    if (count == 0 && first)
    {
      convery.lock();
      convery.init();
      convery.unlock();
    }
    if (convery.ifready())
    {
      convery.lock();
      for (int i = 0; i < num_of_split; ++i)
      {
        cloud_rst[i].header = convery.cloud_list_[i].header;
        for (unsigned int index = 0; index< convery.cloud_list_[i].points.size(); index++)
        {
          cloud_rst[i].points.push_back(convery.cloud_list_[i].points[index]);
        }
      }
      convery.clear();
      convery.unlock();
      count++;
    }
    if (count > 0 && count % 10 == 0 && first)
    {
      convery.lock();
      if (count != 50)
      {
        cloud_rst[count/10].points.clear();
        convery.unlock();
      }
      else 
      {
        first = false;
        chatter_pub.publish(cloud_rst[0]);
        cloud_rst[0].points.clear();
        cloud_rst.push_back(cloud_rst[0]);
        cloud_rst.erase(cloud_rst.begin());
        convery.setbase();
        count = 0;
      }
      convery.unlock();
    }
    else if (count == 10 && !first)
    {
      convery.lock();
      chatter_pub.publish(cloud_rst[0]);
      cloud_rst[0].points.clear();
      cloud_rst.push_back(cloud_rst[0]);
      cloud_rst.erase(cloud_rst.begin());
      convery.setbase();
      count = 0;
      convery.unlock();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}