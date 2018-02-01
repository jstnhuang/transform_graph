#ifndef _TRANSFORM_GRAPH_ORIENTATION_H_
#define _TRANSFORM_GRAPH_ORIENTATION_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

namespace transform_graph {
/// \brief Orientation provides conversions from common orientation types.
///
/// By default, it is the identity orientation.
/// This class provides a number of methods to convert back to common types.
class Orientation {
 public:
  /// Default constructor, identity rotation.
  Orientation();
  /// Quaternion constructor.
  Orientation(double w, double x, double y, double z);
  /// Eigen rotation matrix constructor.
  Orientation(const Eigen::Matrix3d& m);
  /// Eigen::Quaterniond constructor.
  Orientation(const Eigen::Quaterniond& q);
  /// geometry_msgs quaternion constructor.
  Orientation(const geometry_msgs::Quaternion& q);
  /// tf quaternion constructor.
  Orientation(const tf::Quaternion& q);
  /// tf rotation matrix constructor.
  Orientation(const tf::Matrix3x3& m);

  /// \brief The rotation matrix represented by this orientation.
  /// \returns The rotation matrix.
  Eigen::Matrix3d matrix() const;

  /// \brief The quaternion representation of this orientation.
  /// \returns The quaternion.
  Eigen::Quaterniond quaternion() const;

  /// \brief Returns the orientation as a geometry_msgs::Quaternion.
  /// \returns The orientation as a geometry_msgs::Quaternion.
  geometry_msgs::Quaternion quaternion_msg() const;

  /// \brief Returns the orientation as a tf::Quaternion.
  /// \returns The orientation as a tf::Quaternion.
  tf::Quaternion tf_quaternion() const;

  /// \brief Returns the orientation as a tf::Matrix3x3.
  /// \returns The orientation as a tf::Matrix3x3.
  tf::Matrix3x3 tf_matrix() const;

 private:
  Eigen::Matrix3d matrix_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_ORIENTATION_H_
