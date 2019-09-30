#include <vfh3d/vfh3d_planner.h>

namespace vfh3d {

VFH3DPlanner::VFH3DPlanner() {
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

void VFH3DPlanner::poseCb(const geometry_msgs::PoseConstPtr& pose_msg) {
  vehicle_state_->poseCb(pose_msg);
}

void VFH3DPlanner::goalCb(const geometry_msgs::PoseConstPtr& goal_msg) {
  tf::poseMsgToTF(*goal_msg, goal_);
}

void VFH3DPlanner::octomapCb(const octomap_msgs::OctomapConstPtr& octomap_msg) {
  auto new_tree = static_cast<OcTree*>(octomap_msgs::binaryMsgToMap(*octomap_msg));
  oc_tree_->swapContent(*new_tree);
  delete new_tree;

  update();
}

void VFH3DPlanner::update() {

}

}