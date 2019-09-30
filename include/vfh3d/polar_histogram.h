#pragma once
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <vfh3d/vehicle_state.h>
#include <vfh3d/utils.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

using namespace octomap;

namespace vfh3d {

class PolarHistogram {
 public:
  PolarHistogram(
    const std::shared_ptr<OcTree>& oc_tree,
    const std::weak_ptr<VehicleState>& vehicle_state);
  ~PolarHistogram() {}

  void update(const double& max_plan_range, const double& hist_resolution);

  visualization_msgs::MarkerArray getBbxMarkers() {
    return bbx_markers_;
  }

private:

  std::shared_ptr<OcTree> oc_tree_;
  std::weak_ptr<VehicleState> vehicle_state_;
  int marker_id_;
  visualization_msgs::Marker cell_marker_;
  visualization_msgs::MarkerArray bbx_markers_;
};

}