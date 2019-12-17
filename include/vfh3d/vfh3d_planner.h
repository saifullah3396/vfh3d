#pragma once

#include <memory>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <vfh3d/utils.h>
#include <vfh3d/vehicle_state.h>
#include <vfh3d/polar_histogram.h>
#include <vfh3d/CorrectTarget.h>

using namespace octomap;

namespace vfh3d {

class VFH3DPlanner 
{
public:
  VFH3DPlanner();
  ~VFH3DPlanner() {}

  void update();

  // callbacks
  void poseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void cmdPoseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void octomapCb(const octomap_msgs::OctomapConstPtr& octomap_msg);

  // services
  bool correctTarget(
    vfh3d::CorrectTarget::Request& req,
    vfh3d::CorrectTarget::Response& res);

private:
  void updateHistogram();

  // ros
  ros::NodeHandle nh_;

  // publishers
  #ifdef BUILD_WITH_VISUALIZATION
  ros::Publisher bbx_cells_pub_;
  ros::Publisher hist_grid_pub_;
  ros::Publisher updated_direction_pub_;
  #endif
  ros::Publisher cmd_pose_fixed_pub_;

  // subscribers
  ros::Subscriber vehicle_pose_sub_, pose_cmd_sub_, octomap_sub_;
  bool pose_recieved_ = {false};
  bool histogram_updated_ = {false};

  // service
  ros::ServiceServer planner_service_;

  // params
  double map_resolution_;
  
  // planning
  geometry_msgs::PoseStamped cmd_pose_;
  geometry_msgs::PoseStamped cmd_pose_fixed_;
  ros::Time last_pose_stamp_;
  std::shared_ptr<OcTree> oc_tree_;
  std::shared_ptr<VehicleState> vehicle_state_;
  std::unique_ptr<PolarHistogram> polar_histogram_;
};

}