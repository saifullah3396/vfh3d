#include <vfh3d/utils.h>

namespace vfh3d {
namespace utils {

  template <typename T>
  T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
  }
  template int clamp<int>(const int&, const int&, const int&);

  template<typename T>
  T rangeToPi(const T& angle)
  {
    if (angle > M_PI) return angle - M_TWICE_PI;
    else if (angle < -M_PI) return angle + M_TWICE_PI;
    return angle;
  }
  template float rangeToPi<float>(const float& angle);
  template double rangeToPi<double>(const double& angle);

  template<typename T>
  T diffAngle(const T& a1, const T& a2)
  {
    auto a1_r = rangeToPi(a1);
    auto a2_r = rangeToPi(a2);
    if (a1_r * a2_r >= 0) return a1_r - a2_r;
    if (a1_r < 0) return M_TWICE_PI + a1_r - a2_r;
    return a1_r - (M_TWICE_PI + a2_r);
  }
  template float diffAngle<float>(const float& a1, const float& a2);
  template double diffAngle<double>(const double& a1, const double& a2);

  template<typename T>
  T addAngles(const T& a1, const T& a2)
  {
    auto a1_r = rangeToPi(a1);
    auto a2_r = rangeToPi(a2);
    auto ca1 = (a1_r < 0) ? a1_r + M_TWICE_PI : a1_r;
    auto ca2 = (a2_r < 0) ? a2_r + M_TWICE_PI : a2_r;
    return rangeToPi(ca1 + ca2);
  }
  template float addAngles<float>(const float& a1, const float& a2);
  template double addAngles<double>(const double& a1, const double& a2);

  template<typename T>
  T dist(const T& x1, const T& y1, const T& x2, const T& y2)
  {
    auto dx = x1 - x2;
    auto dy = y1 - y2;
    return sqrt(dx*dx + dy*dy);
  }
  template float dist<float>(const float& x1, const float& y1, const float& x2, const float& y2);
  template double dist<double>(const double& x1, const double& y1, const double& x2, const double& y2);

  template<typename T>
  T angleDist(const T& x1, const T& y1, const T& x2, const T& y2) {
    auto dx = utils::diffAngle(x1, x2);
    auto dy = utils::diffAngle(y1, y2);
    return sqrt(dx*dx + dy*dy);
  }
  template float angleDist<float>(const float& x1, const float& y1, const float& x2, const float& y2);
  template double angleDist<double>(const double& x1, const double& y1, const double& x2, const double& y2);

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