#include "accumulater.h"

Accumulater::Accumulater(int n)
{
  transform_base_list_.resize(n);
  cloud_list_.resize(n);
  num_of_split_ = n;
  ready_ = false;
}

void Accumulater::init()
{
  tf::StampedTransform transform;
  listener_.waitForTransform("/world", "/girona500",
                               ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform("/world", "/girona500",
                             ros::Time(0), transform);
  transform_base_list_.resize(num_of_split_, transform);
}

void Accumulater::setbase()
{
  tf::StampedTransform transform;
  listener_.waitForTransform("/world", "/girona500",
                               ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform("/world", "/girona500",
                             ros::Time(0), transform);  
  transform_base_list_.erase(transform_base_list_.begin());
  transform_base_list_.push_back(transform);
}

void Accumulater::copy_pointcloud(std::vector<sensor_msgs::PointCloud> & pcl)
{
  lock();
  for (int i = 0; i < num_of_split_; ++i)
  {
    pcl[i].header = cloud_list_[i].header;
    for (unsigned int index = 0; index< cloud_list_[i].points.size(); index++)
    {
      pcl[i].points.push_back(cloud_list_[i].points[index]);
    }
  }
  clear();
  unlock();
}


void Accumulater::callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
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
  for (int i = 0; i < num_of_split_; ++i)
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

void Accumulater::transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform)
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
