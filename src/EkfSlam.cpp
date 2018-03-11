#include "EkfSlam.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

EkfSlam::EkfSlam() {
    state_ = Eigen::VectorXd::Zero(3);
    covariance_ = Eigen::MatrixXd::Zero(3, 3);
    num_ = 0;   

    predict_covariance_ = Eigen::Matrix3d::Identity() * 0.001;
}

Eigen::Vector3d EkfSlam::merge_predict(Eigen::Vector2d dis, Eigen::Quaterniond q) {
    Eigen::Vector3d update_vec;
    double yaw = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];
    update_vec(0) = dis(0);
    update_vec(1) = dis(1);
    update_vec(2) = yaw;

    return update_vec;
}

void EkfSlam::predict(Eigen::Vector2d dis, Eigen::Quaterniond q, double det_t) {
    Eigen::MatrixXd f = Eigen::MatrixXd::Identity(3 + 3 * num_, 3 + 3 * num_);
    f(2, 2) = 0;
    Eigen::MatrixXd fx = Eigen::MatrixXd::Zero(3, 3 + 3 * num_);
    fx.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d fc = predict_covariance_;
    fc(0, 0) = fc(0, 0) * det_t;
    fc(1, 1) = fc(1, 1) * det_t;

    Eigen::Vector3d update_vec = merge_predict(dis, q);

    state_ = f * state_ + fx.transpose() * update_vec;
    covariance_ = f * covariance_ * f + fx.transpose() * fc * fx;
}

void EkfSlam::update(CloudPair cloud_pair) {
    pcl::PointCloud<pcl::PointXYZ> cloud = cloud_pair.cloud;
    filter(cloud);
    Eigen::Vector2d pos = cloud_pair.pos;

    cloud_vec_.push_back(cloud);

    ROS_DEBUG_STREAM("update!");

    num_++;
    Eigen::VectorXd state_temp(3 + 3 * num_);
    state_temp.head(3 * num_) = state_;
    state_temp(3 * num_) = pos(0);
    state_temp(3 * num_ + 1) = pos(1);
    state_temp(3 * num_ + 2) = 0;
    state_ = state_temp;

    Eigen::MatrixXd covariance_temp = Eigen::MatrixXd::Zero(3 + 3 * num_, 3 + 3 * num_);
    covariance_temp.block(0, 0, 3 * num_, 3 * num_) = covariance_;
    covariance_temp.block(3 * num_, 3 * num_, 3, 3) = covariance_.block(0, 0, 3, 3);
    covariance_temp.block(3 * num_, 0, 3, 3) = covariance_.block(0, 0, 3, 3);
    covariance_temp.block(0, 3 * num_, 3, 3) = covariance_.block(0, 0, 3, 3);
    if (num_ > 1) {
        covariance_temp.block(3*num_, 3, 3, 3 * num_ - 3) = covariance_.block(0, 3, 3, 3 * num_ - 3);
        covariance_temp.block(3, 3*num_, 3 * num_ - 3, 3) = covariance_.block(3, 0, 3 * num_ - 3, 3);
    }
    covariance_ = covariance_temp;

    ROS_DEBUG_STREAM('\n' << state_);

    std::vector<int> close_list;

    close_list = search_nearest();

    for (auto it = close_list.begin(); it != close_list.end(); ++it) {
        Eigen::Vector2d det;
        double sigma;
        det = icp(*it, sigma);
        Eigen::Vector3d det3;
        det3 << det(0), det(1), 0.0;
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov(2,2) = 0;
        cov = cov * sigma;

        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, 3 + 3 * num_);
        h.block(0, *it * 3, 3, 3) = Eigen::Matrix3d::Identity();
        h.block(0, num_ * 3, 3, 3) = Eigen::Matrix3d::Identity();

        Eigen::MatrixXd k = Eigen::MatrixXd::Zero(3 + 3 * num_, 3);
        Eigen::MatrixXd temp = h * covariance_ * h.transpose() + cov;
        k = covariance_ * h.transpose() * temp.inverse();

        state_ = state_ + k * det3;

        Eigen::MatrixXd k_minus = Eigen::MatrixXd::Identity(3 + 3 * num_, 3 + 3 * num_);
        k_minus = k_minus - k * h;

        covariance_ = k_minus * covariance_ * k_minus.transpose() + k * cov * k.transpose();
    }
}

std::vector<int> EkfSlam::search_nearest() {
    std::vector<int> rst;
    rst.clear();
    Eigen::Vector2d current;
    current << state_[3 * num_], state_[3 * num_ + 1];
    for (int i = 0; i < num_ - 1; ++i) {
        Eigen::Vector2d det;
        det << state_[3 + 3 * i], state_[4 + 3 * i];
        det = det - current;
        if (det.norm() < 1) {
            rst.push_back(i);
        }
    }
    return rst;
}

Eigen::Vector2d EkfSlam::icp(int ref, double& sigma) {
    ROS_DEBUG_STREAM("icp: " << ref << " " << num_);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_vec_[ref].makeShared());
    icp.setInputTarget(cloud_vec_[num_ - 1].makeShared());

    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<pcl::PointXYZ> final;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    Eigen::Vector3f v_ref;
    Eigen::Vector3f v_cur;
    v_ref << state_[ref*3+3], state_[ref*3+4], 0.0;
    v_cur << state_[3*num_], state_[3*num_+1], 0.0;
    trans.translation() = v_cur - v_ref;
    icp.align(final, trans.matrix());
    Eigen::Matrix4f matrix = icp.getFinalTransformation();
    sigma = icp.getFitnessScore(0.05);
    Eigen::Vector2d det_icp;
    det_icp << matrix(0, 3), matrix(1, 3);

    return det_icp;
}

pcl::PointCloud<pcl::PointXYZ> EkfSlam::get_cloud() {
    pcl::PointCloud<pcl::PointXYZ> rst, temp;
    for (int i = 0; i < cloud_vec_.size(); ++i) {
        Eigen::Affine3d pos = Eigen::Affine3d::Identity();
        pos.translation() << state_[3*i+3], state_[3*i+4], 0.0;
        pcl::transformPointCloud(cloud_vec_[i], temp, pos);
        rst += temp;
    }
    filter(rst);
    return rst;
}

void EkfSlam::filter(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(cloud);
}
