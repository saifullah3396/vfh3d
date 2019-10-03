#pragma once

#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <tf/tf.h>

namespace vfh3d { 
namespace utils {

  octomath::Vector3 tfToOctomath(const tf::Vector3& tf);
  double getYaw(const tf::Quaternion& q);
  tf::Vector3 getRPY(const tf::Quaternion& q);

}
}