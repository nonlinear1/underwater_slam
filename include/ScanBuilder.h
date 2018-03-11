#ifndef PROJECT_SCANBUILDER_H
#define PROJECT_SCANBUILDER_H
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <boost/thread/mutex.hpp>

struct LaserPair{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Affine3d det_pos;
};

struct CloudPair{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector2d pos;
};

class ScanBuilder {

public:

    ScanBuilder();

    void grab_laser(const sensor_msgs::LaserScan::ConstPtr &scan_in);

    void update_pos(Eigen::Vector2d det, Eigen::Vector2d dis, Eigen::Quaterniond q);

    bool already_finish(){
        bool rst;
        bool_mutex_.lock();
        if (finish_)
        {
            finish_ = false;
            rst = true;
        }
        else
        {
            rst = false;
        }
        bool_mutex_.unlock();
        return rst;
    }

    CloudPair get_cloud(){return cloud_pair_;}

private:
    std::vector<LaserPair> laser_vector_;
    Eigen::Vector2d current_det_pos_;
    Eigen::Vector2d det_pos_;
    Eigen::Vector2d start_pos_;
    Eigen::Quaterniond current_q_;
    CloudPair cloud_pair_;
    boost::mutex pos_mutex_;
    boost::mutex bool_mutex_;
    bool first_;
    bool finish_;
};


#endif //PROJECT_SCANBUILDER_H
