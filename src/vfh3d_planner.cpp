#include <vfh3d/vfh3d_planner.h>
#include <visualization_msgs/MarkerArray.h>

namespace vfh3d {

VFH3DPlanner::VFH3DPlanner() {
  // get params
  auto p_nh = ros::NodeHandle("~");
  std::string octomap_topic, pose_topic;
  double hist_resolution, max_plan_range, const_a;
  p_nh.getParam("octomap_topic", octomap_topic);
  p_nh.getParam("pose_topic", pose_topic);
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
  octomap_sub_ = 
    nh_.subscribe<octomap_msgs::Octomap>(
      octomap_topic, 10, &VFH3DPlanner::octomapCb, this);
  
  // Initialize service for updating input target with local plan
  std::string service_name = "/vfh3d/correct_target";
  planner_service_ = nh_.advertiseService(service_name, &VFH3DPlanner::correctTarget, this);

  #ifdef BUILD_WITH_VISUALIZATION
  bbx_cells_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vfh3d/bbx_cells", 10);
  hist_grid_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vfh3d/hist_grid", 10);
  #endif
}

void VFH3DPlanner::poseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  vehicle_state_->poseCb(pose_msg);
  pose_recieved_ = true;
}

bool VFH3DPlanner::correctTarget(
  vfh3d::CorrectTarget::Request& req,
  vfh3d::CorrectTarget::Response& res)
{
  if (histogram_updated_) {
    auto target_vel = 
      tf::Vector3(req.target_vel.twist.linear.x, req.target_vel.twist.linear.y, req.target_vel.twist.linear.z);
    auto planned_vel = polar_histogram_->windowSearch(target_vel);
    res.corrected_vel.header.frame_id = "map";
    res.corrected_vel.header.stamp = ros::Time::now();
    res.corrected_vel.twist.linear.x = planned_vel.x();
    res.corrected_vel.twist.linear.y = planned_vel.y();
    res.corrected_vel.twist.linear.z = planned_vel.z();
    return true;
  } else {
    ROS_WARN("Polar Histogram not initialized yet. Cannot correct velocity.");
    return false;
  }
}

void VFH3DPlanner::octomapCb(const octomap_msgs::OctomapConstPtr& octomap_msg) {
  auto new_tree = static_cast<OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg));
  oc_tree_->swapContent(*new_tree);
  delete new_tree;
  if (pose_recieved_) // update histogram on every octomap callback
    update();
}

void VFH3DPlanner::update() {
  polar_histogram_->update();
  #ifdef BUILD_WITH_VISUALIZATION
    bbx_cells_pub_.publish(polar_histogram_->getBbxMarkers());
    hist_grid_pub_.publish(polar_histogram_->getDataMarkers());
  #endif
  histogram_updated_ = true;
}

}