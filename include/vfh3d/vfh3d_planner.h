#pragma once

#include <memory>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <tf/tf.h>
#include <vfh3d/utils.h>
#include <vfh3d/vehicle_state.h>

namespace vfh3d {

class VFH3DPlanner 
{
public:
  VFH3DPlanner() {
    vehicle_state = std::unique_ptr<VehicleState>(
      new VehicleState(tf::Vector3(1.0, 1.0, 1.0)));
  }
  ~VFH3DPlanner() {}

private:
  std::unique_ptr<VehicleState> vehicle_state;
};

}