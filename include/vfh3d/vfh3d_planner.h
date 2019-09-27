#pragma once

#include <memory>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <vfh3d/utils.h>
#include <vfh3d/vehicle_state.h>

using namespace octomap;

namespace vfh3d {

class VFH3DPlanner 
{
public:
  VFH3DPlanner() {
    // get params
    auto p_nh = ros::NodeHandle("~");
    p_nh.getParam("map_resolution", map_resolution_);
    p_nh.getParam("max_plan_range", max_plan_range_);
    p_nh.getParam("alpha", alpha_);

    std::string robot_description;
    p_nh.getParam("robot_description", robot_description);
    urdf::Model model;
    if (model.initString(robot_description)) {
      ROS_FATAL("Failed to parse vehicle urdf.");
      return;
    }
    auto collision_box = 
      std::static_pointer_cast<urdf::Box>(model.getLink("base_link")->collision->geometry)->dim;
    auto vehicle_bbox = tf::Vector3(collision_box.x, collision_box.y, collision_box.z);
    oc_tree_ = std::unique_ptr<OcTree>(new OcTree(map_resolution_));
    vehicle_state_ = std::unique_ptr<VehicleState>(new VehicleState(vehicle_bbox));

    // Initialize subscribers
    vehicle_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("pose_in", 10, &VFH3DPlanner::poseCb, this);
    goal_sub_ = nh_.subscribe<geometry_msgs::Pose>("goal_in", 10, &VFH3DPlanner::goalCb, this);
    octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("octomap_in", 10, &VFH3DPlanner::octomapCb, this);

    // Initialize publishers
    histogram_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("polar_histogram", 10);
    planned_target_pub_ = nh_.advertise<geometry_msgs::Pose>("planned_target", 10);
  }
  ~VFH3DPlanner() {}

  void poseCb(const geometry_msgs::PoseConstPtr& pose_msg) {
    vehicle_state_->poseCb(pose_msg);
  }

  void goalCb(const geometry_msgs::PoseConstPtr& goal_msg) {
    tf::poseMsgToTF(*goal_msg, goal_);
  }

  void octomapCb(const octomap_msgs::OctomapConstPtr& octomap_msg) {
    oc_tree_->swapContent(*static_cast<OcTree*>(octomap_msgs::binaryMsgToMap(*octomap_msg)));
  }

private:
  // ros
  ros::NodeHandle nh_;
  tf::Pose goal_;

  // publishers
  ros::Publisher histogram_pub_, planned_target_pub_;

  // subscribers
  ros::Subscriber vehicle_pose_sub_, octomap_sub_, goal_sub_;

  // params
  double alpha_;
  double max_plan_range_;
  double map_resolution_;

  // planning
  std::unique_ptr<OcTree> oc_tree_;
  std::unique_ptr<VehicleState> vehicle_state_;
};

}