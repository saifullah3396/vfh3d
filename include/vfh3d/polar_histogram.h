#pragma once
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
    const double& resolution);
  ~PolarHistogram() {}

  void update(const double& max_plan_range);
  bool inSphere(const octomap::point3d& p, const double& radius);

  visualization_msgs::MarkerArray getBbxMarkers() {
    return bbx_markers_;
  }

  visualization_msgs::MarkerArray getDataMarkers() {
    return data_markers_;
  }

private:

  class HistVoxel {
  public:
    HistVoxel(
      const octomap::point3d& position,
      const float& occ_value,
      PolarHistogram* histogram) :
      position_(position),
      occ_value_(occ_value),
      histogram_(histogram)
    {
      vehicle_position_ = histogram_->vehicle_state_.lock()->center();
      dist_from_vehicle_ = (position - vehicle_position_).norm();
    }

    double dist() const { return dist_from_vehicle_; }
    void updateHist() {
      // compute weight
      computeWeight();

      auto diff = position_ - vehicle_position_;
      // azimuth angle histogram x - coordinate
      int bz = floor((int)(atan2(diff.y(), diff.x()) / histogram_->resolution_)) + histogram_->data_.cols() / 2.0;

      // elevation angle histogram y - coordinate
      int be = 
        floor((int)(
          atan2(diff.z(), sqrt(diff.x()*diff.x() + diff.y()*diff.y())) /
          histogram_->resolution_)) + 
          histogram_->data_.rows() / 2.0;
      
      be = std::min(std::max(be, 0), (int)histogram_->data_.rows());
      bz = std::min(std::max(bz, 0), (int)histogram_->data_.cols());
      // each voxel affects multiple histogram bins within its size range
      auto oneOverRes = 1.0 / histogram_->resolution_;
      min_max_angle_ = floor(oneOverRes * asin(std::min(histogram_->enlargement_radius_ / dist_from_vehicle_, 1.0)));
      auto r_min = std::max((int)(be - min_max_angle_), 0);
      auto r_max = std::min((int)(be + min_max_angle_), (int)histogram_->data_.rows());
      auto c_min = std::max((int)(bz - min_max_angle_), 0);
      auto c_max = std::min((int)(bz + min_max_angle_), (int)histogram_->data_.cols());
      for (int r = r_min; r < r_max; ++r) {
        for (int c = c_min; c < c_max; ++c) {
          if (histogram_->data_(r, c) == histogram_->data_(r, c))
            histogram_->data_(r, c) += weight_;
          else 
            histogram_->data_(r, c) = weight_;
        }
      }
    }

  private:
    void computeWeight() {
      auto min_dist_from_vehicle = dist_from_vehicle_ - histogram_->enlargement_radius_;
      weight_ = pow(occ_value_, 2) * (histogram_->const_a_ - histogram_->const_b_ * min_dist_from_vehicle);
    }

    octomap::point3d position_, vehicle_position_;
    float occ_value_;
    double dist_from_vehicle_;
    double min_max_angle_;
    double weight_;
    PolarHistogram* histogram_;
  };

  double histSum() {
    double sum = 0.0;
    for (int i = 0; i < data_.rows(); ++i) {
      for (int j = 0; j < data_.cols(); ++j) {
        const auto& d = data_(i, j);
        if (d != d)
          continue;
        sum += d;
      }
    }
    return sum;
  }

  double histMean() {
    return histSum() / data_.size();
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
  size_t height_, width_;
  double enlargement_radius_;
  double voxel_radius_;
  double const_a_, const_b_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> data_;

  std::shared_ptr<OcTree> oc_tree_;
  std::weak_ptr<VehicleState> vehicle_state_;
  int marker_id_;
  visualization_msgs::Marker cell_marker_;
  visualization_msgs::Marker point_marker_;
  visualization_msgs::MarkerArray bbx_markers_;
  visualization_msgs::MarkerArray data_markers_;
};

}