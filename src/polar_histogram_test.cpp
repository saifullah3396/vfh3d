#include <ros/ros.h>
#include <tf/tf.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <vfh3d/CorrectTarget.h>

using namespace octomap;

octomap::OcTree* oc_tree_;
ros::Publisher octomap_pub_, pose_pub_, target_vel_res_pub_, target_vel_pub_;
ros::ServiceClient target_vel_client_;
double resolution_ = 0.15;
double tol_ = 0.001;

void updateNodes(
  const OcTree* oc_tree,
  const int& x_min,
  const int& x_max,
  const int& y_min,
  const int& y_max,
  const int& z_min,
  const int& z_max,
  const bool& occupied) 
{
  for (int i = x_min; i < x_max; ++i) {
    for (int j = y_min; j < y_max; ++j) {
      for (int k = z_min; k < z_max; ++k) {
        auto p = octomap::point3d((i+1) * resolution_ - tol_, (j+1) * resolution_ - tol_, k * resolution_ - tol_);
        oc_tree_->updateNode(p, occupied);
      }
    }
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octobbx_iter_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");
    p_nh.getParam("resolution", resolution_);
    std::string octomap_topic, pose_topic;
    p_nh.getParam("octomap_topic", octomap_topic);
    p_nh.getParam("pose_topic", pose_topic);
    auto rate = ros::Rate(10);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
    target_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/vfh3d/target_vel", 10); 
    target_vel_res_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/vfh3d/corrected_vel", 10); 
    target_vel_client_ = nh.serviceClient<vfh3d::CorrectTarget>("/vfh3d/correct_target");
    oc_tree_ = new octomap::OcTree(resolution_);
    auto max = 20;
    auto min = 5;
    updateNodes(oc_tree_, -10, max, -max, max, -5, 5, false);
    updateNodes(oc_tree_, -max, -min, -max, -min, -5, 5, true);
    updateNodes(oc_tree_, -max, -min, min, max, 0, 1, true);
    updateNodes(oc_tree_, min, max, min, max, -5, 5, true);
    updateNodes(oc_tree_, min, max, -max, -min, 0, 1, true);

    oc_tree_->updateInnerOccupancy();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    geometry_msgs::TwistStamped target_vel;
    target_vel.twist.linear.x = 1.0;
    target_vel.twist.linear.y = 1.0;
    target_vel.header.frame_id = "map";
    vfh3d::CorrectTarget correct_target;
    correct_target.request.target_vel = target_vel;
    while(ros::ok()) {
      target_vel.header.stamp = ros::Time::now();
      octomap_msgs::Octomap pub_msg;
      pub_msg.header.frame_id = "map";
      octomap_msgs::fullMapToMsg<octomap::OcTree>(*oc_tree_, pub_msg);
      octomap_pub_.publish(pub_msg);
      pose_pub_.publish(pose);
      if (target_vel_client_.call(correct_target)) {
        target_vel_pub_.publish(target_vel);
        correct_target.response.corrected_vel.header.frame_id = "map";
        target_vel_res_pub_.publish(correct_target.response.corrected_vel);
      }
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}