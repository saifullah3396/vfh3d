#include <vfh3d/vfh3d_planner.h>
#include <visualization_msgs/MarkerArray.h>

namespace vfh3d {

VFH3DPlanner::VFH3DPlanner() {
  // get params
  auto p_nh = ros::NodeHandle("~");
  std::string octomap_topic, pose_topic, target_vel_topic;
  double hist_resolution, max_plan_range, const_a;
  p_nh.getParam("octomap_topic", octomap_topic);
  p_nh.getParam("pose_topic", pose_topic);
  p_nh.getParam("target_vel_topic", target_vel_topic);
  p_nh.getParam("map_resolution", map_resolution_);
  p_nh.getParam("max_plan_range", max_plan_range);
  p_nh.getParam("const_a", const_a);
  p_nh.getParam("hist_resolution", hist_resolution);

  double vehicle_safety_radius;
  double turning_radius_l;
  double turning_radius_r;
  p_nh.getParam("vehicle_safety_radius", vehicle_safety_radius);
  p_nh.getParam("turning_radius_l", turning_radius_l);
  p_nh.getParam("turning_radius_r", turning_radius_r);

  std::string robot_description;
  nh_.getParam("robot_description", robot_description);
  urdf::Model model;
  if (!model.initString(robot_description)) {
    ROS_FATAL("Failed to parse vehicle urdf.");
    return;
  }
  auto collision_box = 
    std::static_pointer_cast<urdf::Box>(
      model.getLink("base_link_inertia")->collision->geometry)->dim;
  auto vehicle_bbox = 
    tf::Vector3(collision_box.x, collision_box.y, collision_box.z);
  oc_tree_ = 
    std::shared_ptr<OcTree>(new OcTree(map_resolution_));
  vehicle_state_ = 
    std::shared_ptr<VehicleState>(
        new VehicleState(
            vehicle_bbox, 
            vehicle_safety_radius, 
            turning_radius_l, 
            turning_radius_r));
  polar_histogram_ = 
    std::unique_ptr<PolarHistogram>(
        new PolarHistogram(
          oc_tree_, vehicle_state_, hist_resolution, max_plan_range, const_a));

  // Initialize subscribers
  vehicle_pose_sub_ = 
    nh_.subscribe<geometry_msgs::PoseStamped>(
      pose_topic, 10, &VFH3DPlanner::poseCb, this);
  target_vel_sub_ = 
    nh_.subscribe<geometry_msgs::TwistStamped>(
      target_vel_topic, 10, &VFH3DPlanner::targetCb, this);
  octomap_sub_ = 
    nh_.subscribe<octomap_msgs::Octomap>(
      octomap_topic, 10, &VFH3DPlanner::octomapCb, this);

  // Initialize publishers
  planned_target_vel_pub_ = 
    nh_.advertise<geometry_msgs::TwistStamped>(
      "/vfh3d/planned_target_vel", 10);
  
  #ifdef BUILD_WITH_VISUALIZATION
  bbx_cells_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vfh3d/bbx_cells", 10);
  hist_grid_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vfh3d/hist_grid", 10);
  #endif
}

void VFH3DPlanner::poseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  vehicle_state_->poseCb(pose_msg);
  pose_recieved_ = true;
}

void VFH3DPlanner::targetCb(const geometry_msgs::TwistStampedConstPtr& target_vel_msg) {
  auto target_vel = 
    tf::Vector3(target_vel_msg->twist.linear.x, target_vel_msg->twist.linear.y, target_vel_msg->twist.linear.z);
  polar_histogram_->setTargetVel(target_vel);
  if (pose_recieved_ && octomap_recieved_)
    update();
}

void VFH3DPlanner::octomapCb(const octomap_msgs::OctomapConstPtr& octomap_msg) {
  auto new_tree = static_cast<OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg));
  oc_tree_->swapContent(*new_tree);
  delete new_tree;
  octomap_recieved_ = true;
}

void VFH3DPlanner::update() {
  polar_histogram_->update();
  auto planned_vel = polar_histogram_->getUpdatedVel();
  geometry_msgs::TwistStamped planned_vel_msg;
  planned_vel_msg.header.frame_id = "map";
  planned_vel_msg.header.stamp = ros::Time::now();
  planned_vel_msg.twist.linear.x = planned_vel.x();
  planned_vel_msg.twist.linear.y = planned_vel.y();
  planned_vel_msg.twist.linear.z = planned_vel.z();
  planned_target_vel_pub_.publish(planned_vel_msg);
  #ifdef BUILD_WITH_VISUALIZATION
    bbx_cells_pub_.publish(polar_histogram_->getBbxMarkers());
    hist_grid_pub_.publish(polar_histogram_->getDataMarkers());
  #endif
}

}