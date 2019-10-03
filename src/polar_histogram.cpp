#include <vfh3d/polar_histogram.h>

namespace vfh3d {

PolarHistogram::PolarHistogram(
  const std::shared_ptr<OcTree>& oc_tree,
  const std::weak_ptr<VehicleState>& vehicle_state,
  const double& resolution,
  const double& max_plan_range,
  const int& window_size_x,
  const int& window_size_y) :
  oc_tree_(oc_tree),
  vehicle_state_(vehicle_state),
  resolution_(resolution),
  max_plan_range_(max_plan_range),
  window_size_x_(window_size_x),
  window_size_y_(window_size_y)
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
  point_marker_ = cell_marker_;
  point_marker_.type = visualization_msgs::Marker::CUBE;
  point_marker_.scale.x = res / 5.0;
  point_marker_.scale.y = res / 5.0;
  point_marker_.scale.z = res / 5.0;
  point_marker_.color.r = 1.0;
  marker_id_ = 0;

  auto v = vehicle_state_.lock();
  // in paper, it is robot_radius + safety_radius + voxel_size. voxel_size should be voxel_radius?
  voxel_radius_ = oc_tree_->getResolution() * 0.5;
  enlargement_radius_ = 
    v->getRadius() + v->getSafetyRadius() + voxel_radius_;
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

  const_a_ = 0.5;
  const_b_ = (const_a_ - 1) / pow((max_plan_range - 1.0) / 2.0, 2);

  // matrix padding
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

void PolarHistogram::update() {
  bbx_markers_.markers.clear();
  auto v = vehicle_state_.lock();
  auto range = octomath::Vector3(max_plan_range_, max_plan_range_, max_plan_range_);
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
    if (voxel.dist() > max_plan_range_) // not in sphere
      continue;
    // add weight of each voxel in histogram bins affected by it
    voxel.updateHist();

    /*if (iter->getValue() > 0) { // high probablity of occupied
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
    bbx_markers_.markers.push_back(cell_marker_);*/
  }

  auto mean = histMean();
  auto std = histStd(mean);
  data_markers_.markers.clear();
  auto t_low = mean - std;
  auto t_high = mean + std;
  auto point_arrow_size = tf::Vector3(1, 0, 0);
  bool ground_in_range = v->getPose().getOrigin().z() - enlargement_radius_ <= 0;
  for (int i = 0; i < height_; ++i) {
    for (int j = 0; j < width_; ++j) {
      // If robot height is close to ground
      if (ground_in_range && i < height_half_) {
        data_(i, j) = 1.0;      
      } else {
        const auto& d = data_(i, j);
        if (d != d) { // if it is unseen
          // If robot height is close to ground
          //if (j > 0) data_(i, j) = data_(i, j-1);
          //else data_(i, j) = 0.0;
          data_(i, j) = 0.0;
        } else if (d > t_high) {
          data_(i, j) = 1.0;
        } else if (d < t_low) {
          data_(i, j) = 0.0;
        } else {
          if (j > 0) data_(i, j) = data_(i, j-1);
          else data_(i, j) = 0.0;
        }
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
      //data_markers_.markers.push_back(point_marker_);
    }
  }

  padded_data_.block(pad_rows_, pad_cols_, height_, width_) = data_;
  padded_data_.block(pad_rows_, 0, height_, pad_cols_) = data_.rightCols(pad_cols_);
  padded_data_.block(pad_rows_, padded_data_.cols() - pad_cols_, height_, pad_cols_) = data_.leftCols(pad_cols_);
  //padded = (padded.array().isNaN()).select(padded, 1); // set Nan values to occupied since we don't know
  // find target velocity direction and angles
  auto t_direction = target_vel_.normalized();
  auto t_yaw = computeAzimuthAngle(t_direction);
  auto t_pitch = computeElevationAngle(t_direction);
  auto robot_yaw = v->getYaw();
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
        

        tf::Transform t;
        t.setRotation(tf::createQuaternionFromRPY(0, pitch, yaw));
        t.setOrigin(tf::Vector3(v->center().x(), v->center().y(), v->center().z()));
        auto point = t * point_arrow_size;
        point_marker_.color.b = 1.0;
        point_marker_.color.r = 0.0;
        point_marker_.color.g = 0.0;
        point_marker_.pose.position.x = point.x();
        point_marker_.pose.position.y = point.y();
        point_marker_.pose.position.z = point.z();
        point_marker_.pose.orientation.x = t.getRotation().x();
        point_marker_.pose.orientation.y = t.getRotation().y();
        point_marker_.pose.orientation.z = t.getRotation().z();
        point_marker_.pose.orientation.w = t.getRotation().w();
        point_marker_.id = marker_id_++;
        data_markers_.markers.push_back(point_marker_);
      } 
    }
  }

  if (best_i != -1) {
    best_yaw_ = data_yaw_lut_(best_i, best_j);
    best_pitch_ = data_pitch_lut_(best_i, best_j);
    tf::Transform t;
    t.setRotation(tf::createQuaternionFromRPY(0, best_pitch_, best_yaw_));
    t.setOrigin(tf::Vector3(v->center().x(), v->center().y(), v->center().z()));
    auto point = t * point_arrow_size;
    point_marker_.color.b = 0.0;
    point_marker_.color.r = 0.0;
    point_marker_.color.g = 1.0;
    point_marker_.pose.position.x = point.x();
    point_marker_.pose.position.y = point.y();
    point_marker_.pose.position.z = point.z();
    point_marker_.pose.orientation.x = t.getRotation().x();
    point_marker_.pose.orientation.y = t.getRotation().y();
    point_marker_.pose.orientation.z = t.getRotation().z();
    point_marker_.pose.orientation.w = t.getRotation().w();
    point_marker_.id = marker_id_++;
    data_markers_.markers.push_back(point_marker_);
  }
}

}