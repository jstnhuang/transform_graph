#ifndef _STF_ORIENTATION_H_
#define _STF_ORIENTATION_H_

#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace stf {
// Orientation provides conversions from common orientation types.
class Orientation {
 public:
  Orientation(double w, double x, double y, double z);
  Orientation(const Eigen::Matrix3d& m);
  Orientation(const Eigen::Quaterniond& q);
  Orientation(const geometry_msgs::Quaternion& q);
  Orientation(const tf::Quaternion& q);
  Orientation(const tf::Matrix3x3& m);

  Eigen::Matrix3d matrix() const;

 private:
  Eigen::Matrix3d matrix_;
};
}  // namespace stf

#endif  // _STF_ORIENTATION_H_
