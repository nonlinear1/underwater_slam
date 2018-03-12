#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "show_path");

    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Publisher pub_path = node.advertise<nav_msgs::Path>("groud_ture", 1);

    ros::Rate rate(60.0);

    nav_msgs::Path path_rst;
    path_rst.header.stamp = ros::Time::now();
    path_rst.header.seq++;
    path_rst.header.frame_id = "world";

    while (node.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("world", "multibeam",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header = path_rst.header;
        tf::Vector3 origin = transform.getOrigin();
        pose.pose.position.x = origin.x();
        pose.pose.position.y = origin.y();
        pose.pose.position.z = 0;
        tf::quaternionTFToMsg(transform.getRotation(), pose.pose.orientation);

        path_rst.poses.push_back(pose);
        pub_path.publish(path_rst);

        rate.sleep();
    }
    return 0;
};
