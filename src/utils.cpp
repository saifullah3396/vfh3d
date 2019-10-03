#include <vfh3d/utils.h>

namespace vfh3d {
namespace utils {

  octomath::Vector3 tfToOctomath(const tf::Vector3& tf) {
    return octomath::Vector3(tf.getX(), tf.getY(), tf.getZ());
  }

  double getYaw(const tf::Quaternion& q) {
    double r, p, y;
    tf::Matrix3x3(q).getRPY(r, p, y);
    return y;
  }

  tf::Vector3 getRPY(const tf::Quaternion& q) {
    double r, p, y;
    tf::Matrix3x3(q).getRPY(r, p, y);
    return tf::Vector3(r, p, y);
  }

}
}