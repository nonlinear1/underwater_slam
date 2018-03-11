#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "ScanBuilder.h"
#include "EkfSlam.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "underwater_slam/RequireControl.h"

void resolve_control(Eigen::Vector2d& dis, Eigen::Quaterniond& q, underwater_slam::RequireControl srv, double det_t)
{
    Eigen::Vector2d velocity;
    Eigen::Vector2d velocity_g;
    double yaw;

    yaw = srv.response.yaw;
    velocity << srv.response.linear_velocity.x, srv.response.linear_velocity.y;
    q.w() = srv.response.orientation.w;
    q.vec() << srv.response.orientation.x, srv.response.orientation.y, srv.response.orientation.z;

    velocity_g(0) = velocity(0) * cos(yaw) - velocity(1) * sin(yaw);
    velocity_g(1) = velocity(0) * sin(yaw) + velocity(1) * cos(yaw);

    dis = velocity_g*det_t;
}

double calculate_time(ros::Time& t)
{
    ros::Time current = ros::Time::now();
    ros::Duration duration = current - t;
    double det;

    t = current;
    det = duration.toSec();

    return det;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam");
    ros::NodeHandle node;


    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ScanBuilder scan_builder;
    EkfSlam ekf_slam;

    underwater_slam::RequireControl control_srv;

    ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("g500/multibeam", 10, &ScanBuilder::grab_laser,
                                                                 &scan_builder);
    ros::ServiceClient control_client = node.serviceClient<underwater_slam::RequireControl>("request_control");

    ros::Publisher pub_path = node.advertise<nav_msgs::Path>("Boatpath", 1);
    ros::Publisher pub_points = node.advertise<pcl::PointCloud<pcl::PointXYZ>>("Points", 1);

    ros::Rate loop_rate(60);

    ros::Time time_now = ros::Time::now();
    double det_t;

    Eigen::Vector2d dis;
    Eigen::Quaterniond q;

    nav_msgs::Path path_rst;
    path_rst.header.stamp = ros::Time::now();
    path_rst.header.seq++;
    path_rst.header.frame_id = "world";

    while (ros::ok()) {

        det_t = calculate_time(time_now);

        if (!control_client.call(control_srv)) {
            ROS_ERROR("Failed to call service");
            return 1;
        }

        resolve_control(dis, q, control_srv, det_t);

        ekf_slam.predict(dis, q, det_t);
        scan_builder.update_pos(ekf_slam.get_current_state(), dis, q);

        if (scan_builder.already_finish()) {
            ekf_slam.update(scan_builder.get_cloud());
            pcl::PointCloud<pcl::PointXYZ> cloud = ekf_slam.get_cloud();
            cloud.header.frame_id = "world";
            pub_points.publish(cloud);
            ROS_DEBUG_STREAM("publish :" << cloud.size());
        }

        geometry_msgs::PoseStamped pose;
        pose.header = path_rst.header;
        Eigen::Vector2d state = ekf_slam.get_current_state();
        pose.pose.position.x = state(0);
        pose.pose.position.y = state(1);
        pose.pose.position.z = 0;
        pose.pose.orientation = control_srv.response.orientation;

        path_rst.poses.push_back(pose);
        pub_path.publish(path_rst);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

