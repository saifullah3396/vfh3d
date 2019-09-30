#pragma once
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>
#include <vfh3d/vehicle_state.h>
#include <vfh3d/utils.h>
#include <geometry_msgs/Pose.h>

using namespace octomap;

namespace vfh3d {

class PolarHistogram {
 public:
  PolarHistogram(
    const std::shared_ptr<OcTree>& oc_tree,
    const std::weak_ptr<VehicleState>& vehicle_state) :
    oc_tree_(oc_tree),
    vehicle_state_(vehicle_state)
  {}
  ~PolarHistogram() {}

  void update(const double& max_plan_range, const double& hist_resolution) {
    auto v = vehicle_state_.lock();
    auto range = octomath::Vector3(max_plan_range, max_plan_range, max_plan_range);
    auto min_hist_range = v->min() - range;
    auto max_hist_range = v->max() + range;
    // in paper, it is robot_radius + safety_radius + voxel_size. voxel_size should be voxel_radius?
    auto voxel_enlargement_radius = 
      v->getRadius() + v->getSafetyRadius() + oc_tree_->getResolution() * 0.5;

    // iterate through the nodes in the box
    auto begin = oc_tree_->begin_leafs_bbx(min_hist_range, max_hist_range);
    for(auto iter = begin; iter!= oc_tree_->end_leafs_bbx(); ++iter)
    {
      // first test if this is correct
    }
  }

private:

  std::shared_ptr<OcTree> oc_tree_;
  std::weak_ptr<VehicleState> vehicle_state_;
};

}