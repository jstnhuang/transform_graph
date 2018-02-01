#ifndef _TRANSFORM_GRAPH_TRANSFORM_H_
#define _TRANSFORM_GRAPH_TRANSFORM_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_datatypes.h"

#include "transform_graph/orientation.h"
#include "transform_graph/position.h"

namespace transform_graph {
/// \brief Transform describes a combination of a rotation and a translation.
///
/// By default it is the identity transform.
///
/// There are various methods for converting a Transform back into other common
/// types, such as tf::Tranform, geometry_msgs::Pose, and Eigen::Affine3d.
class Transform {
 public:
  /// Default, identity constructor.
  Transform();

  /// Constructor from other transform_graph types (or implicitly convertible
  /// types).
  ///
  /// \param[in] position The position of this frame in the coordinate system of
  ///   the reference frame.
  /// \param[in] orientation The orientation of this frame relative to the
  ///   reference frame.
  Transform(const transform_graph::Position& position,
            const transform_graph::Orientation& orientation);
  /// Constructor from a tf Transform.
  Transform(const tf::Transform& st);
  /// Constructor from a pose.
  Transform(const geometry_msgs::Pose& p);
  /// Constructor from a pose.
  Transform(const geometry_msgs::Transform& p);
  /// Constructor from an Eigen Affine matrix.
  Transform(const Eigen::Affine3d& a);
  /// Constructor from an Eigen matrix.
  Transform(const Eigen::Matrix4d& m);

  /// \deprecated
  /// Returns the identity transform.
  static Transform Identity();

  /// \brief Returns the position component of this transform.
  //
  /// \returns The position component of this transform.
  transform_graph::Position position() const;

  /// \brief Returns the orientation component of this transform.
  //
  /// \returns The orientation component of this transform.
  transform_graph::Orientation orientation() const;

  /// \brief Returns this transform as a tf::Transform.
  /// \returns This transform as a tf::Transform.
  tf::Transform tf_transform() const;

  /// \brief Returns this transform as a geometry_msgs::Pose.
  /// \returns This transform as a Pose.
  geometry_msgs::Pose pose() const;

  /// \brief Returns this transform as a geometry_msgs::Pose.
  /// \returns This transform as a Pose.
  geometry_msgs::Transform transform() const;

  /// \brief Returns this transform's homogeneous transform matrix.
  /// \returns This transform's homogeneous transform matrix.
  Eigen::Matrix4d matrix() const;

  /// \brief Returns this transform as an affine transformation matrix.
  /// \returns This transform's affine transformation matrix.
  Eigen::Affine3d affine() const;

  /// \deprecated
  /// \brief Returns this transform as a geometry_msgs::Pose.
  /// \returns This transform as a Pose.
  void ToPose(geometry_msgs::Pose* pose) const;

  /// \brief Returns the inverse of this transform.
  /// \returns The inverse of this transform.
  transform_graph::Transform inverse() const;

 private:
  Eigen::Affine3d transform_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_TRANSFORM_H_
