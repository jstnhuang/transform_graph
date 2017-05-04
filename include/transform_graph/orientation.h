#ifndef _TRANSFORM_GRAPH_ORIENTATION_H_
#define _TRANSFORM_GRAPH_ORIENTATION_H_

#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace transform_graph {
/// \brief Orientation provides conversions from common orientation types.
///
/// By default it is the identity orientation.
/// To get the rotation matrix, call Orientation::matrix().
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

 private:
  Eigen::Matrix3d matrix_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_ORIENTATION_H_
