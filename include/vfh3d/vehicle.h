#pragma once

#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <tf/tf.h>
#include <vfh3d/utils.h>

namespace vfh3d {

class Vehicle 
{
public:
  Vehicle(
    const geometry_msgs::Pose& init_pose, const tf::Vector3& size) :
    size_(size)
  {
    tf::poseMsgToTF(init_pose, pose_);
    radius_ = size_[size_.maxAxis()];
    auto size_half = size_ * 0.5;
    min_ = pose_.getOrigin() - size_half;
    max_ = pose_.getOrigin() + size_half;
  }

  ~Vehicle() {}

  void poseCb(const geometry_msgs::PoseConstPtr& pose) {
    last_pose_ = pose_;
    tf::poseMsgToTF(*pose, pose_);
    auto size_half = size_ * 0.5;
    min_ = pose_.getOrigin() - size_half;
    max_ = pose_.getOrigin() + size_half;
  }

  tf::Pose getLastPose() const { return last_pose_; }
  tf::Pose getPose() const { return pose_; }
  double getTurningRadiusR() { return turning_radius_r_;}
  double getTurningRadiusL() { return turning_radius_l_;}
  double getRadius() const { return radius_; }
  double getSafetyRadius() const { return safety_radius_; }
  octomath::Vector3 center() const { return utils::tfToOctomath(pose_.getOrigin()); }
  octomath::Vector3 min() const { return utils::tfToOctomath(min_); }
  octomath::Vector3 max() const { return utils::tfToOctomath(max_); }
  double getYaw() const { return utils::getYaw(pose_.getRotation()); }

private:
  tf::Pose pose_; // vehicle pose
  tf::Pose last_pose_; // last vehicle pose
  tf::Vector3 size_; // vehicle size in 3d
  tf::Vector3 min_; // vehicle minimum boundary in 3d
  tf::Vector3 max_; // vehicle maximum boundary in 3d
  double radius_; // vehicle radius
  double turning_radius_l_ = {0.2}; // maximum left turn radius
  double turning_radius_r_ = {0.2}; // maximum right turn radius
  double safety_radius_; // safety radius
};

}