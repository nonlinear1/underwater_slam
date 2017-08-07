#include "ros/ros.h"
#include <math.h>
#include <boost/thread.hpp>
#include "sensor_msgs/PointCloud.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define WIDE 1200
#define HEIGHT 20

cv::Mat image;
bool ready = false;
boost::mutex mutex;

void scanCallback (const sensor_msgs::PointCloud::ConstPtr& point_in)
{
  int wide = WIDE;
  int len = point_in->points.size()/WIDE;
  image = cv::Mat(wide, len, CV_8UC1);
  for (int i = 0; i < wide; i++)
  {
    for (int j = 0; j < len; ++j)
    {
      image.at<uchar>(wide,len) = uchar(127*(point_in->points[j*wide+i].z/10+1));
    }
  }
  mutex.lock();
  ready = true;
  mutex.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_img");

  ros::NodeHandle n_sub;
  ros::NodeHandle n_adv;
  image_transport::ImageTransport it(n_adv);
  image_transport::Publisher pub = it.advertise("/image", 1);

  ros::Subscriber sub = n_sub.subscribe("PointCloudAdd", 1, scanCallback);
  ros::Rate loop_rate(1000);
  
  while (ros::ok())
  {
  	mutex.lock();
    if (ready)
    {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
      pub.publish(msg);
    }
    mutex.unlock();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}