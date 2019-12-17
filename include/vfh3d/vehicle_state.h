#pragma once

#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <tf/tf.h>
#include <vfh3d/utils.h>

namespace vfh3d {

class VehicleState 
{
public:
  VehicleState(
    const tf::Vector3& size,
    const double& safety_radius,
    const double& turning_radius_l = 0.2,
    const double& turning_radius_r = 0.2) :
    size_(size),
    safety_radius_(safety_radius),
    turning_radius_l_(turning_radius_l),
    turning_radius_r_(turning_radius_r)
  {
    radius_ = size_[size_.maxAxis()] / 2.0;
  }

  ~VehicleState() {}

  void poseCb(const geometry_msgs::PoseStampedConstPtr& pose) {
    tf::poseMsgToTF((*pose).pose, pose_);
    auto size_half = size_ * 0.5;
    min_ = pose_.getOrigin() - size_half;
    max_ = pose_.getOrigin() + size_half;
  }

  tf::Pose getPose() const { return pose_; }
  double getTurningRadiusR() { return turning_radius_r_;}
  double getTurningRadiusL() { return turning_radius_l_;}
  double getRadius() const { return radius_; }
  double getSafetyRadius() const { return safety_radius_; }
  octomath::Vector3 center() const { return utils::tfToOctomath(pose_.getOrigin()); }
  octomath::Vector3 min() const { return utils::tfToOctomath(min_); }
  octomath::Vector3 max() const { return utils::tfToOctomath(max_); }
  double getYaw() const { return utils::getYaw(pose_.getRotation()); }
  tf::Vector3 getRPY() const { return utils::getRPY(pose_.getRotation()); }

private:
  tf::Pose pose_; // vehicle pose
  tf::Vector3 size_; // vehicle size in 3d
  tf::Vector3 min_ = {tf::Vector3(0.0, 0.0, 0.0)}; // vehicle minimum boundary in 3d
  tf::Vector3 max_ = {tf::Vector3(0.0, 0.0, 0.0)}; // vehicle maximum boundary in 3d
  double radius_; // vehicle radius
  double turning_radius_l_; // maximum left turn radius
  double turning_radius_r_; // maximum right turn radius
  double safety_radius_; // safety radius
};

}