#include "transform_graph/position.h"

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"

namespace transform_graph {
Position::Position() : vector_() { vector_ << 0, 0, 0; }

Position::Position(double x, double y, double z) : vector_() {
  vector_ << x, y, z;
}

Position::Position(const Eigen::Vector3d& v) : vector_() { vector_ = v; }

Position::Position(const geometry_msgs::Point& p) : vector_() {
  vector_ << p.x, p.y, p.z;
}

Position::Position(const geometry_msgs::Vector3& v) : vector_() {
  vector_ << v.x, v.y, v.z;
}

Position::Position(const pcl::PointXYZ& p) : vector_() {
  vector_ << p.x, p.y, p.z;
}

Position::Position(const tf::Vector3& v) : vector_() {
  vector_ << v.x(), v.y(), v.z();
}

Eigen::Vector3d Position::vector() const { return vector_; }

geometry_msgs::Point Position::point() const {
  geometry_msgs::Point point;
  point.x = vector_.x();
  point.y = vector_.y();
  point.z = vector_.z();
  return point;
}

geometry_msgs::Vector3 Position::vector_msg() const {
  geometry_msgs::Vector3 point;
  point.x = vector_.x();
  point.y = vector_.y();
  point.z = vector_.z();
  return point;
}

pcl::PointXYZ Position::pcl_point() const {
  pcl::PointXYZ point;
  point.x = vector_.x();
  point.y = vector_.y();
  point.z = vector_.z();
  return point;
}

tf::Vector3 Position::tf_vector3() const {
  tf::Vector3 point(vector_.x(), vector_.y(), vector_.z());
  return point;
}

double Position::x() const { return vector_.x(); }

double Position::y() const { return vector_.y(); }

double Position::z() const { return vector_.z(); }
}  // namespace transform_graph
