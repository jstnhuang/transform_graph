#ifndef _STF_TRANSFORM_H_
#define _STF_TRANSFORM_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

#include "stf/orientation.h"
#include "stf/position.h"

namespace stf {
/// \brief Transform describes a combination of a rotation and a translation.
///
/// By default it is the identity transform.
/// To get the homogeneous transform matrix, call Transform::matrix().
/// The top left 3x3 submatrix is the rotation matrix.
/// The columns of the rotation matrix describe the principal axes of the local
/// frame relative to the reference frame.
/// The top right 3x1 vector is the origin of the local frame relative to the
/// reference frame.
class Transform {
 public:
  /// Default, identity constructor.
  Transform();
  /// Constructor from other stf types (or implicitly convertible types).
  Transform(const stf::Position& position, const stf::Orientation& orientation);
  /// Constructor from a tf Transform.
  Transform(const tf::Transform& st);
  /// Constructor from a pose.
  Transform(const geometry_msgs::Pose& p);
  /// Constructor from an Eigen Affine matrix.
  Transform(const Eigen::Affine3d& a);
  /// Constructor from an Eigen matrix.
  Transform(const Eigen::Matrix4d& m);

  /// Returns the identity transform.
  static Transform Identity();

  /// \brief Returns this transform's homogeneous transform matrix.
  /// \returns This transform's homogeneous transform matrix.
  Eigen::Matrix4d matrix() const;

  /// \brief Returns the inverse of this transform.
  /// \returns The inverse of this transform.
  stf::Transform inverse() const;

 private:
  Eigen::Affine3d transform_;
};
}  // namespace stf

#endif  // _STF_TRANSFORM_H_
