#include <vfh3d/polar_histogram.h>

namespace vfh3d {

PolarHistogram::PolarHistogram(
  const std::shared_ptr<OcTree>& oc_tree,
  const std::weak_ptr<VehicleState>& vehicle_state,
  const double& resolution) :
  oc_tree_(oc_tree),
  vehicle_state_(vehicle_state),
  resolution_(resolution)
{
  cell_marker_.header.frame_id = "map";
  cell_marker_.ns = "fbe3d";
  cell_marker_.type = visualization_msgs::Marker::CUBE;
  cell_marker_.action = visualization_msgs::Marker::MODIFY;
  cell_marker_.color.a = 0.5;
  cell_marker_.lifetime = ros::Duration(10);
  auto res = oc_tree->getResolution();
  cell_marker_.scale.x = res;
  cell_marker_.scale.y = res;
  cell_marker_.scale.z = res;
  point_marker_ = cell_marker_;
  point_marker_.type = visualization_msgs::Marker::CUBE;
  point_marker_.scale.x = res / 3.0;
  point_marker_.scale.y = res / 3.0;
  point_marker_.scale.z = res / 3.0;
  point_marker_.color.r = 1.0;
  marker_id_ = 0;

  auto v = vehicle_state_.lock();
  // in paper, it is robot_radius + safety_radius + voxel_size. voxel_size should be voxel_radius?
  voxel_radius_ = oc_tree_->getResolution() * 0.5;
  enlargement_radius_ = 
    v->getRadius() + v->getSafetyRadius() + voxel_radius_;
  height_ = size_t(ceil(M_PI / resolution_)); // steps in radians
  width_ = size_t(ceil(2 * M_PI / resolution_)); // steps in radians
  data_.resize(height_, width_);
  data_.setConstant(NAN);

  const_a_ = 0.5;
  const_b_ = (const_a_ - 1) / pow((v->getRadius() - 1.0) / 2.0, 2);
}

void PolarHistogram::update(const double& max_plan_range) {
  bbx_markers_.markers.clear();
  auto v = vehicle_state_.lock();
  auto range = octomath::Vector3(max_plan_range, max_plan_range, max_plan_range);
  auto min_hist_range = v->min() - range;
  auto max_hist_range = v->max() + range;

  std_msgs::ColorRGBA color;
  color.a = 1;
  color.r = 1;
  cell_marker_.color = color;
  // iterate through the nodes in the box
  auto begin = oc_tree_->begin_leafs_bbx(min_hist_range, max_hist_range);
  auto end = oc_tree_->end_leafs_bbx();
  marker_id_ = 0;

  // reset histogram
  data_.setConstant(NAN);
  for(auto iter = begin; iter!= end; ++iter)
  {
    auto voxel = HistVoxel(iter.getCoordinate(), iter->getOccupancy(), this);
    if (voxel.dist() > max_plan_range) // not in sphere
      continue;
    // add weight of each voxel in histogram bins affected by it
    voxel.updateHist();

    if (iter->getValue() > 0) { // high probablity of occupied
      color.r = 0;
      color.b = 1;
      cell_marker_.color = color;
    } else { // else
      color.r = 1;
      color.b = 0;
      cell_marker_.color = color;
    }

    auto coord = iter.getCoordinate();
    cell_marker_.pose.position.x = coord.x();
    cell_marker_.pose.position.y = coord.y();
    cell_marker_.pose.position.z = coord.z();
    cell_marker_.id = marker_id_++;
    bbx_markers_.markers.push_back(cell_marker_);
  }

  data_markers_.markers.clear();
  auto t_high = 0.6;
  auto t_low = 0.4;
  for (int i = 0; i < data_.rows(); ++i) {
    for (int j = 0; j < data_.cols(); ++j) {
      const auto& d = data_(i, j);
      if (d != d) {
        point_marker_.color.a = 1.0;
        point_marker_.color.b = 0.0;
        point_marker_.color.r = 1.0;
        point_marker_.color.g = 0.0;
        auto x = (i - data_.rows() / 2) * max_plan_range * resolution_;
        auto y = (j - data_.cols() / 2) * max_plan_range * resolution_;
        point_marker_.pose.position.x = v->center().x() + x * cos(v->getYaw()) - y * sin(v->getYaw());
        point_marker_.pose.position.y = v->center().y() + x * sin(v->getYaw()) + y * cos(v->getYaw());
        point_marker_.pose.position.z = v->center().z();
        point_marker_.pose.orientation.x = v->getPose().getRotation().x();
        point_marker_.pose.orientation.y = v->getPose().getRotation().y();
        point_marker_.pose.orientation.z = v->getPose().getRotation().z();
        point_marker_.pose.orientation.w = v->getPose().getRotation().w();
        point_marker_.id = marker_id_++;
        data_markers_.markers.push_back(point_marker_);
        continue;
      }
      if (d > t_high) {
        data_(i, j) = 1.0;
      } else if (d < t_low) {
        data_(i, j) = 0.0;
      } else {
        data_(i, j) = fabs(d - t_low) < fabs(d - t_high) ? 0.0 : 1.0;
      }
      if (data_(i, j) > 0.5) {
        point_marker_.color.a = 1.0;
        point_marker_.color.b = 1.0;
        point_marker_.color.r = 1.0;
        point_marker_.color.g = 1.0;
      } else {
        point_marker_.color.a = 1.0;
        point_marker_.color.b = 0.0;
        point_marker_.color.r = 0.0;
        point_marker_.color.g = 0.0;
      }
      auto x = (i - data_.rows() / 2) * max_plan_range * resolution_;
      auto y = (j - data_.cols() / 2) * max_plan_range * resolution_;
      point_marker_.pose.position.x = v->center().x() + x * cos(v->getYaw()) - y * sin(v->getYaw());
      point_marker_.pose.position.y = v->center().y() + x * sin(v->getYaw()) + y * cos(v->getYaw());
      point_marker_.pose.position.z = v->center().z();
      point_marker_.pose.orientation.x = v->getPose().getRotation().x();
      point_marker_.pose.orientation.y = v->getPose().getRotation().y();
      point_marker_.pose.orientation.z = v->getPose().getRotation().z();
      point_marker_.pose.orientation.w = v->getPose().getRotation().w();
      point_marker_.id = marker_id_++;
      data_markers_.markers.push_back(point_marker_);
    }
  }
}

}