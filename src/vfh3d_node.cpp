#include <ros/ros.h>
#include <tf/tf.h>
#include <vfh3d/vfh3d_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vfh3d_node");
    vfh3d::VFH3DPlanner planner;
    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}