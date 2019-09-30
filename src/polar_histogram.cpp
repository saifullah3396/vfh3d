#include <vfh3d/polar_histogram.h>

namespace vfh3d {

PolarHistogram::PolarHistogram(
  const std::shared_ptr<OcTree>& oc_tree,
  const std::weak_ptr<VehicleState>& vehicle_state) :
  oc_tree_(oc_tree),
  vehicle_state_(vehicle_state)
{
  cell_marker_.header.frame_id = "map";
  cell_marker_.ns = "fbe3d";
  cell_marker_.type = visualization_msgs::Marker::CUBE;
  cell_marker_.action = visualization_msgs::Marker::MODIFY;
  cell_marker_.color.a = 0.5;
  cell_marker_.lifetime = ros::Duration(0.1);
  auto res = oc_tree->getResolution();
  cell_marker_.scale.x = res;
  cell_marker_.scale.y = res;
  cell_marker_.scale.z = res;
  marker_id_ = 0;
}

void PolarHistogram::update(const double& max_plan_range, const double& hist_resolution) {
  bbx_markers_.markers.clear();
  auto v = vehicle_state_.lock();
  auto range = octomath::Vector3(max_plan_range, max_plan_range, max_plan_range);
  auto min_hist_range = v->min() - range;
  auto max_hist_range = v->max() + range;
  // in paper, it is robot_radius + safety_radius + voxel_size. voxel_size should be voxel_radius?
  auto voxel_enlargement_radius = 
    v->getRadius() + v->getSafetyRadius() + oc_tree_->getResolution() * 0.5;

  std_msgs::ColorRGBA color;
  color.a = 1;
  color.r = 1;
  cell_marker_.color = color;
  // iterate through the nodes in the box
  auto begin = oc_tree_->begin_leafs_bbx(min_hist_range, max_hist_range);
  auto end = oc_tree_->end_leafs_bbx();
  marker_id_ = 0;        
  for(auto iter = begin; iter!= end; ++iter)
  {
    auto coord = iter.getCoordinate();
    cell_marker_.pose.position.x = coord.x();
    cell_marker_.pose.position.y = coord.y();
    cell_marker_.pose.position.z = coord.z();
    cell_marker_.id = marker_id_++;
    bbx_markers_.markers.push_back(cell_marker_);
  }
}

}