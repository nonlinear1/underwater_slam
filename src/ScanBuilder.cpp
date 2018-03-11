#include "ScanBuilder.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <pcl/common/transforms.h>

ScanBuilder::ScanBuilder() {
    current_det_pos_ = Eigen::Vector2d::Zero();
    det_pos_ = Eigen::Vector2d::Zero();
    first_ = true;
    finish_ = false;
}

void ScanBuilder::grab_laser(const sensor_msgs::LaserScan::ConstPtr &scan_in) {
    LaserPair laser_pair;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pos_mutex_.lock();
    if (current_det_pos_.norm() < 0.1) {
        pos_mutex_.unlock();
        return;
    }
    laser_pair.det_pos = Eigen::Affine3d::Identity();
    laser_pair.det_pos.translation() << current_det_pos_(0), current_det_pos_(1), 0.0;
    laser_pair.det_pos.rotate(current_q_);
    det_pos_ += current_det_pos_;
    current_det_pos_ = Eigen::Vector2d::Zero();
    pos_mutex_.unlock();

    boost::numeric::ublas::matrix<double> ranges(2, scan_in->ranges.size() - 1);
    boost::numeric::ublas::matrix<double> angle(2, scan_in->ranges.size() - 1);

    double z_max = 0;
    double angle_min = scan_in->angle_min;
    double angle_max = scan_in->angle_max;
    double angle_increment = scan_in->angle_increment;

    for (unsigned int index = 0; index < scan_in->ranges.size() - 1; index++) {
        angle(0, index) = cos(angle_min + (double) index * angle_increment);
        angle(1, index) = sin(angle_min + (double) index * angle_increment);
    }
    for (unsigned int index = 0; index < scan_in->ranges.size() - 1; index++) {
        ranges(0, index) = (double) scan_in->ranges[index];
        ranges(1, index) = (double) scan_in->ranges[index];
    }
    boost::numeric::ublas::matrix<double> output = element_prod(ranges, angle);
    for (unsigned int index = 0; index < scan_in->ranges.size() - 1; index++) {
        pcl::PointXYZ point(0.0, output(1, index), -output(0, index));
        cloud.push_back(point);
    }
    laser_pair.cloud = cloud;
    laser_vector_.push_back(laser_pair);

    if (det_pos_.norm() > 4) {
        ROS_DEBUG_STREAM("det_pos: " << det_pos_.norm());
        pos_mutex_.lock();
        first_ = true;
        pos_mutex_.unlock();

        det_pos_ = Eigen::Vector2d::Zero();

        cloud_pair_.cloud.clear();
        unsigned long num = laser_vector_.size();
        unsigned long mid = num / 2;

        std::vector<Eigen::Affine3d> transform_vec;
        transform_vec.resize(num);
        Eigen::Affine3d start = Eigen::Affine3d::Identity();
        start.translation() << start_pos_[0], start_pos_[1], 0.0;
        transform_vec[0] = start * laser_vector_[0].det_pos;
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        pcl::transformPointCloud(laser_vector_[0].cloud, cloud_out, transform_vec[0]);
        for (int i = 1; i < num; ++i) {
            transform_vec[i] = laser_vector_[i].det_pos;
            transform_vec[i].translation() += transform_vec[i - 1].translation();
        }
        cloud_pair_.pos << transform_vec[mid].translation()(0), transform_vec[mid].translation()(1);
        Eigen::Affine3d inv = Eigen::Affine3d::Identity();
        inv.translation() = transform_vec[mid].translation();
        for (int j = 0; j < num; ++j) {
            transform_vec[j].translation() = transform_vec[j].translation() - inv.translation(); 
            pcl::transformPointCloud(laser_vector_[j].cloud, cloud_out, transform_vec[j]);
            cloud_pair_.cloud += cloud_out;
        }
        ROS_DEBUG_STREAM("finish!");

        laser_vector_.clear();
        bool_mutex_.lock();
        finish_ = true;
        bool_mutex_.unlock();
    }
}

void ScanBuilder::update_pos(Eigen::Vector2d pos, Eigen::Vector2d dis, Eigen::Quaterniond q) {
    pos_mutex_.lock();
    if (first_) {
        start_pos_ = pos;
        ROS_DEBUG_STREAM("start_pos " << start_pos_);
        first_ = false;
    }
    current_det_pos_ += dis;
    current_q_ = q;
    pos_mutex_.unlock();
}
