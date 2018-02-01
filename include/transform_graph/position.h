#ifndef _TRANSFORM_GRAPH_POSITION_H_
#define _TRANSFORM_GRAPH_POSITION_H_

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"

namespace transform_graph {
/// \brief Position provides conversions from common position types.
///
/// By default it is (0, 0, 0).
/// This class also provides various methods to convert back to common types.
class Position {
 public:
  /// Default constructor, (0, 0, 0).
  Position();
  /// x/y/z constructor.
  Position(double x, double y, double z);
  /// Eigen::Vector3d constructor.
  Position(const Eigen::Vector3d& v);
  /// geometry_msgs point constructor.
  Position(const geometry_msgs::Point& p);
  /// geometry_msgs vector constructor.
  Position(const geometry_msgs::Vector3& v);
  /// PCL point constructor.
  Position(const pcl::PointXYZ& p);
  /// tf vector constructor.
  Position(const tf::Vector3& v);

  /// \brief Returns this translation as an Eigen::Vector3d.
  /// \returns The translation, as an Eigen::Vector3d.
  Eigen::Vector3d vector() const;

  /// \brief Returns this translation as geometry_msgs::Point.
  /// \returns The translation, as a geometry_msgs::Point.
  geometry_msgs::Point point() const;

  /// \brief Returns this translation as geometry_msgs::Vector3.
  /// \returns The translation, as a geometry_msgs::Vector3.
  geometry_msgs::Vector3 vector_msg() const;

  /// \brief Returns this translation as pcl::PointXYZ.
  /// \returns The translation, as a pcl::PointXYZ.
  pcl::PointXYZ pcl_point() const;

  /// \brief Returns this translation as tf::Vector3.
  /// \returns The translation, as a tf::Vector3.
  tf::Vector3 tf_vector3() const;

  /// \brief Returns the x component of this position.
  /// \returns The x component of this position.
  double x() const;

  /// \brief Returns the y component of this position.
  /// \returns The y component of this position.
  double y() const;

  /// \brief Returns the z component of this position.
  /// \returns The z component of this position.
  double z() const;

 private:
  Eigen::Vector3d vector_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_POSITION_H_
