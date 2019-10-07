#pragma once
#include <algorithm>
#include <numeric>
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <vfh3d/vehicle_state.h>
#include <vfh3d/utils.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>

using namespace octomap;

namespace vfh3d {

class PolarHistogram {
 public:
  PolarHistogram(
    const std::shared_ptr<OcTree>& oc_tree,
    const std::weak_ptr<VehicleState>& vehicle_state,
    const double& resolution,
    const double& max_plan_range,
    const double& const_a = 10.0,
    const int& window_size_x = 3,
    const int& window_size_y = 3);
  ~PolarHistogram() {}

  void update();
  tf::Vector3 windowSearch(const tf::Vector3& target_vel);
  bool inSphere(const octomap::point3d& p, const double& radius);

  #ifdef BUILD_WITH_VISUALIZATION
  visualization_msgs::MarkerArray getBbxMarkers() const {
    return bbx_markers_;
  }

  visualization_msgs::MarkerArray getDataMarkers() const {
    return data_markers_;
  }
  #endif

private:
  #ifdef BUILD_WITH_VISUALIZATION
  void resetVisualization();
  #endif
  void generateHistogram();
  void binarizeHistogram();

  template <typename Vector3>
  double computeAzimuthAngle(const Vector3& p) const {
    return atan2(p.y(), p.x());
  }

  template <typename Vector3>
  double computeElevationAngle(const Vector3& p) const {
    return atan2(-p.z(), sqrt(p.x()*p.x() + p.y()*p.y()));
  }

  template <typename Vector3>
  int computeDiscreteAzimuthAngle(const Vector3& p) const {
    return 
      utils::clamp(
        (int)(width_half_ - floor(computeAzimuthAngle(p) / resolution_)),
        0, width_ - 1);
  }

  template <typename Vector3>
  int computeDiscreteElevationAngle(const Vector3& p) const {
    return 
      utils::clamp(
        (int)(height_half_ - floor(computeElevationAngle(p) / resolution_)),
        0,
        height_ - 1);
  }

  class HistVoxel {
  public:
    HistVoxel(
      const octomap::point3d& position,
      const float& occ_value,
      const float& size,
      PolarHistogram* histogram) :
      position_(position),
      radius_(size / 2.0),
      occ_value_(occ_value),
      h_(histogram)
    {
      enlargement_radius_ = h_->enlargement_radius_wo_voxel_ + radius_;
      vehicle_position_ = h_->vehicle_state_.lock()->center();
      dist_from_vehicle_ = (position - vehicle_position_).norm();
    }

    void setWeight(const double &weight) { weight_ = weight; }
    double getWeight() const { return weight_; }
    double dist() const { return dist_from_vehicle_; }
    void updateHist() {
      auto diff = position_ - vehicle_position_;
      // azimuth angle histogram x - coordinate
      auto bz = h_->computeDiscreteAzimuthAngle(diff);

      // elevation angle histogram y - coordinate
      auto be = h_->computeDiscreteElevationAngle(diff);

      // each voxel affects multiple histogram bins within its size range
      if (occ_value_ > 0) { // occupied voxel
        // compute weight
        computeWeight(true);
        min_max_angle_ = 
          floor(
            h_->res_inverse_ * 
            asin(std::min(enlargement_radius_ / dist_from_vehicle_, 1.0)));
      } else { // free voxel
        weight_ = 0.0;
        min_max_angle_ = 0;
      }
      if (min_max_angle_ > 0) {
        int r_min = std::max((int)(be - min_max_angle_), 0);
        int r_max = std::min((int)(be + min_max_angle_), h_->height_ - 1);
        int c_min = bz - min_max_angle_;
        int c_max = bz + min_max_angle_;
        for (int r = r_min; r < r_max; ++r) {
          for (int c = c_min; c < c_max; ++c) {
            int wrapped_c = c;
            if (wrapped_c >= h_->width_) {
              wrapped_c = c - h_->width_;
            } else if (wrapped_c < 0) {
              wrapped_c = c + h_->width_;
            }
            if (h_->data_(r, wrapped_c) == h_->data_(r, wrapped_c))
              h_->data_(r, wrapped_c) += weight_;
            else 
              h_->data_(r, wrapped_c) = weight_;
          }
        }
      } else {
        if (h_->data_(be, bz) == h_->data_(be, bz))
          h_->data_(be, bz) += weight_;
        else 
          h_->data_(be, bz) = weight_;
      }
    }

  private:
    void computeWeight(const bool& enlargement) {
      double min_dist_from_vehicle;
      if (enlargement) min_dist_from_vehicle = dist_from_vehicle_ - enlargement_radius_;
      else min_dist_from_vehicle = dist_from_vehicle_ - radius_;
      weight_ = pow(occ_value_, 2) * (h_->const_a_ - h_->const_b_ * min_dist_from_vehicle);
    }

    octomap::point3d position_, vehicle_position_;
    float occ_value_;
    double dist_from_vehicle_;
    double min_max_angle_;
    double weight_;
    double radius_, enlargement_radius_;
    PolarHistogram* h_;
  };

  double histMean() {
    int count = 0;
    double sum = 0.0;
    for (int i = 0; i < data_.rows(); ++i) {
      for (int j = 0; j < data_.cols(); ++j) {
        const auto& d = data_(i, j);
        if (d != d)
          continue;
        sum += d;
        count++;
      }
    }
    return sum / count;
  }

  double histStd(const double& mean) {
    double sum = 0.0;
    for (int i = 0; i < data_.rows(); ++i) {
      for (int j = 0; j < data_.cols(); ++j) {
        const auto& d = data_(i, j);
        if (d != d)
          continue;
        sum += pow(d - mean, 2);
      }
    }
    return sqrt(sum / data_.size() - 1);
  }

  double resolution_;
  double res_inverse_;
  int height_, width_;
  int height_half_, width_half_;
  double enlargement_radius_wo_voxel_;
  double const_a_, const_b_;
  double max_plan_range_;
  int window_size_x_, window_size_y_;
  int pad_rows_, pad_cols_;
  const int max_window_size_ = {7};
  double target_diff_w_ = {5.0};
  double vehicle_diff_w_ = {2.0};
  double prev_target_diff_w_ = {2.0};
  double best_yaw_ = {NAN};
  double best_pitch_ = {NAN};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> data_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> data_pitch_lut_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> data_yaw_lut_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> padded_data_;

  std::shared_ptr<OcTree> oc_tree_;
  std::weak_ptr<VehicleState> vehicle_state_;
  #ifdef BUILD_WITH_VISUALIZATION
  int marker_id_;
  visualization_msgs::Marker cell_marker_;
  visualization_msgs::Marker point_marker_;
  visualization_msgs::MarkerArray bbx_markers_;
  visualization_msgs::MarkerArray data_markers_;
  #endif
};

}