#include <vfh3d/polar_histogram.h>

namespace vfh3d {

PolarHistogram::PolarHistogram(
  const std::shared_ptr<OcTree>& oc_tree,
  const std::weak_ptr<VehicleState>& vehicle_state,
  const double& resolution,
  const double& max_plan_range,
  const double& const_a,
  const int& window_size_x,
  const int& window_size_y) :
  oc_tree_(oc_tree),
  vehicle_state_(vehicle_state),
  resolution_(resolution),
  max_plan_range_(max_plan_range),
  const_a_(const_a),
  const_b_(const_a / max_plan_range), // a - bd_max = 0
  window_size_x_(window_size_x),
  window_size_y_(window_size_y)
{
  #ifdef BUILD_WITH_VISUALIZATION
    cell_marker_.header.frame_id = "map";
    cell_marker_.ns = "fbe3d";
    cell_marker_.type = visualization_msgs::Marker::CUBE;
    cell_marker_.action = visualization_msgs::Marker::MODIFY;
    cell_marker_.color.a = 0.5;
    cell_marker_.lifetime = ros::Duration(0.1);
    auto tree_resolution = oc_tree->getResolution();
    cell_marker_.scale.x = tree_resolution;
    cell_marker_.scale.y = tree_resolution;
    cell_marker_.scale.z = tree_resolution;
    point_marker_ = cell_marker_;
    point_marker_.type = visualization_msgs::Marker::CUBE;
    point_marker_.scale.x = tree_resolution / 5.0;
    point_marker_.scale.y = tree_resolution / 5.0;
    point_marker_.scale.z = tree_resolution / 5.0;
    point_marker_.color.r = 1.0;
    marker_id_ = 0;
  #endif

  auto v = vehicle_state_.lock();
  enlargement_radius_wo_voxel_ = v->getRadius() + v->getSafetyRadius();
  res_inverse_ = 1.0 / resolution_;
  height_ = int(floor(M_PI / resolution_)); // steps in radians
  width_ = int(floor(2 * M_PI / resolution_)); // steps in radians
  height_half_ = height_ / 2;
  width_half_ = width_ / 2;
  data_.resize(height_, width_);
  data_.setConstant(NAN);
  data_pitch_lut_.resize(height_, width_);
  data_yaw_lut_.resize(height_, width_);

  for (int i = 0; i < height_; ++i) {
    for (int j = 0; j < width_; ++j) {
      data_pitch_lut_(i, j) = (height_half_ - i) * resolution_;
      data_yaw_lut_(i, j) = (width_half_ - j) * resolution_;
    }
  }

  // matrix padding for moving window search
  assert(window_size_x_ % 2 != 0 && 
         window_size_x_ <= max_window_size_ &&
         "Window size should be an odd integer and less than max_size.");
  assert(window_size_y_ % 2 != 0 && 
         window_size_y_ <= max_window_size_ &&
         "Window size should be an odd integer and less than max_size.");
  pad_rows_ = (window_size_y_ - 1) / 2;
  pad_cols_ = (window_size_x_ - 1) / 2;
  
  padded_data_.resize(height_ + pad_rows_ * 2, width_ + pad_cols_ * 2);
  padded_data_.setZero();
}

#ifdef BUILD_WITH_VISUALIZATION
void PolarHistogram::resetVisualization()
{
  data_markers_.markers.clear();
  bbx_markers_.markers.clear();
  marker_id_ = 0;
}
#endif

void PolarHistogram::generateHistogram()
{
  // reset histogram
  data_.setConstant(NAN);
  #ifdef BUILD_WITH_VISUALIZATION
  std::vector<double> weights;
  cell_marker_.color.a = 0.25;
  #endif
  auto v = vehicle_state_.lock();
  auto range = octomath::Vector3(max_plan_range_, max_plan_range_, max_plan_range_);
  auto min_hist_range = v->min() - range;
  auto max_hist_range = v->max() + range;

  // iterate through the nodes in the box
  auto begin = oc_tree_->begin_leafs_bbx(min_hist_range, max_hist_range);
  auto end = oc_tree_->end_leafs_bbx();
  for(auto iter = begin; iter!= end; ++iter)
  {
    auto voxel = HistVoxel(iter.getCoordinate(), std::max(iter->getValue(), 0.f), iter.getSize(), this);
    if (voxel.dist() > max_plan_range_) // not in sphere
      continue;
    // add weight of each voxel in histogram bins affected by it
    voxel.updateHist();
    //voxel.setWeight(iter->getLogOdds());

    #ifdef BUILD_WITH_VISUALIZATION
    auto coord = iter.getCoordinate();
    cell_marker_.scale.x = iter.getSize();
    cell_marker_.scale.y = iter.getSize();
    cell_marker_.scale.z = iter.getSize();
    cell_marker_.pose.position.x = coord.x();
    cell_marker_.pose.position.y = coord.y();
    cell_marker_.pose.position.z = coord.z();
    cell_marker_.pose.orientation.w = 1.0;
    cell_marker_.id = marker_id_++;
    bbx_markers_.markers.push_back(cell_marker_);
    weights.push_back(voxel.getWeight());
    #endif
  }

  #ifdef BUILD_WITH_VISUALIZATION
  std::vector<double> norm_weights(weights.size());
  double min_w = *std::min_element(weights.begin(), weights.end());
  double max_w = *std::max_element(weights.begin(), weights.end());
  std::transform(
    weights.begin(), 
    weights.end(), 
    norm_weights.begin(), [min_w, max_w](const double& w)
    { return w-min_w/(max_w-min_w); }
  );
  for (int i = 0; i < bbx_markers_.markers.size(); ++i) {
    auto v = norm_weights[i];
    auto& m = bbx_markers_.markers[i];
    if (v < 0.5) { // generate heat map from weights
      m.color.r = 0;
      m.color.g = int( 255./0.5 * v);
      m.color.b = int( 255. + -255./0.5  * v);
    } else {
      m.color.r = int( 255./(1.0-0.5) * (v - 0.5));
      m.color.g = int( 255. + -255./(1.0-0.5)  * (v - 0.5));
      m.color.b = 0;
    }
  }
  #endif
}

void PolarHistogram::binarizeHistogram()
{
  auto v = vehicle_state_.lock();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> data_wo_nan = 
    (data_.array().isNaN()).select(0, data_);
  double hist_min = data_wo_nan.minCoeff();
  double hist_max = data_wo_nan.maxCoeff();
  data_wo_nan = (data_wo_nan.array() - hist_min) / (hist_max - hist_min);
  data_ = (data_.array().isNaN()).select(0, data_wo_nan);
  auto mean = histMean();
  //auto std = histStd(mean);
  auto t_low = mean;
  auto t_high = mean;
  auto point_arrow_size = tf::Vector3(1, 0, 0);
  bool ground_in_range = // is ground within vehicle plan range?
    v->getPose().getOrigin().z() - 
    enlargement_radius_wo_voxel_ <= 0;
  for (int i = 0; i < height_; ++i) {
    for (int j = 0; j < width_; ++j) {
      // If robot height is close to ground
      if (ground_in_range && i < height_half_) {
        data_(i, j) = 1.0;      
      } else {
        const auto& d = data_(i, j);
        if (d >= t_high) {
          data_(i, j) = 1.0;
        } else if (d < t_low) {
          data_(i, j) = 0.0;
        }
      }
      #ifdef BUILD_WITH_VISUALIZATION
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
      tf::Transform t;
      t.setRotation(tf::createQuaternionFromRPY(0, data_pitch_lut_(i, j), data_yaw_lut_(i, j)));
      t.setOrigin(tf::Vector3(v->center().x(), v->center().y(), v->center().z()));
      auto point = t * point_arrow_size;
      point_marker_.pose.position.x = point.x();
      point_marker_.pose.position.y = point.y();
      point_marker_.pose.position.z = point.z();
      point_marker_.pose.orientation.x = t.getRotation().x();
      point_marker_.pose.orientation.y = t.getRotation().y();
      point_marker_.pose.orientation.z = t.getRotation().z();
      point_marker_.pose.orientation.w = t.getRotation().w();
      point_marker_.id = marker_id_++;
      data_markers_.markers.push_back(point_marker_);
      #endif
    }
  }
}

tf::Vector3 PolarHistogram::windowSearch(const tf::Vector3& target_vel)
{
  tf::Vector3 updated_vel;
  padded_data_.block(pad_rows_, pad_cols_, height_, width_) = data_;
  padded_data_.block(pad_rows_, 0, height_, pad_cols_) = data_.rightCols(pad_cols_);
  padded_data_.block(pad_rows_, padded_data_.cols() - pad_cols_, height_, pad_cols_) = data_.leftCols(pad_cols_);
  // find target velocity direction and angles
  auto t_direction = target_vel.normalized();
  auto t_yaw = computeAzimuthAngle(t_direction);
  auto t_pitch = computeElevationAngle(t_direction);
  auto robot_yaw = vehicle_state_.lock()->getYaw();
  double min_weight = 1e6;
  int best_i = -1, best_j = -1;
  auto prev_best_yaw = best_yaw_;
  auto prev_best_pitch = best_pitch_;
  for (int i = pad_rows_; i < height_ + pad_rows_; ++i) {
    for (int j = pad_cols_; j < width_ + pad_cols_; ++j) {
      // occupied if any is greater than 0.5
      auto occupied = 
        padded_data_.block(
          i - pad_rows_, j - pad_cols_, window_size_y_, window_size_x_).array() > 0.5;
      if (!occupied.any()) {
        auto index_i = i - pad_rows_;
        auto index_j = j - pad_cols_;
        const auto& pitch = data_pitch_lut_(index_i, index_j);
        const auto& yaw = data_yaw_lut_(index_i, index_j);
        double weight;
        if (prev_best_pitch == prev_best_pitch) { 
          weight = 
            target_diff_w_ * utils::angleDist(t_yaw, t_pitch, yaw, pitch) + 
            vehicle_diff_w_ * utils::angleDist(robot_yaw, 0.0, yaw, 0.0) + 
            prev_target_diff_w_ * utils::angleDist(prev_best_yaw, prev_best_pitch, yaw, pitch);
        } else {
          weight = 
            target_diff_w_ * utils::angleDist(t_yaw, t_pitch, yaw, pitch) + 
            vehicle_diff_w_ * utils::angleDist(robot_yaw, 0.0, yaw, 0.0);
        }
        if (weight <= min_weight) {
          min_weight = weight;
          best_i = index_i;
          best_j = index_j;
        }
      } 
    }
  }

  if (best_i != -1) {
    best_yaw_ = data_yaw_lut_(best_i, best_j);
    best_pitch_ = data_pitch_lut_(best_i, best_j);
    // update direction
    t_direction[0] = cos(best_yaw_) * cos(best_pitch_);
    t_direction[1] = sin(best_yaw_) * cos(best_pitch_);
    t_direction[2] = -sin(best_pitch_);
    updated_vel = target_vel.length() * t_direction;
  } else {
    updated_vel = target_vel;
    //ROS_ERROR("No solution found for the given target velocity by the vfh3d+ local planner!");
  }

  ROS_DEBUG("target_vel: %f, %f, %f", target_vel.x(), target_vel.y(), target_vel.z());
  ROS_DEBUG("updated_vel: %f, %f, %f", updated_vel.x(), updated_vel.y(), updated_vel.z());
  return updated_vel;
}

void PolarHistogram::update() {
  #ifdef BUILD_WITH_VISUALIZATION
  resetVisualization();
  #endif
  generateHistogram();
  binarizeHistogram();
}

}