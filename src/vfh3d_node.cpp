#include <ros/ros.h>
#include <tf/tf.h>
#include <vfh3d/vehicle.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vfh3d_node");
    geometry_msgs::Pose pose;
    tf::Vector3 size(1.0, 1.0, 1.0);
    vfh3d::Vehicle vehicle(pose, size);
    auto rate = ros::Rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}