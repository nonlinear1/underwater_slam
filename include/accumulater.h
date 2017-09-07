#ifndef ACCUMULATER_H
#define ACCUMULATER_H

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

class Accumulater
{
public:
  Accumulater(int n);
  ~Accumulater(){}
  
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void lock(){mutex_.lock();}
  void unlock(){mutex_.unlock();}
  void ready(){ready_ = true;}
  void clear(){ready_ = false;}
  bool ifready(){return ready_;}
  void setbase();
  void init();
  void copy_pointcloud(std::vector<sensor_msgs::PointCloud> & pcl);
  void transLaser (const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud & cloud_out, tf::StampedTransform& transform);

private:
  std::vector<tf::StampedTransform> transform_base_list_;
  std::vector<sensor_msgs::PointCloud> cloud_list_;
  tf::TransformListener listener_;
  boost::mutex mutex_;
  bool ready_;
  int num_of_split_;
};

#endif //accumulater.h