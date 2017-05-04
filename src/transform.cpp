#include "transform_graph/transform.h"

#include "Eigen/Dense"

#include "transform_graph/position.h"
#include "transform_graph/orientation.h"

using std::string;

namespace transform_graph {
Transform::Transform() : transform_(Eigen::Affine3d::Identity()) {}
Transform::Transform(const transform_graph::Position& position,
                     const transform_graph::Orientation& orientation)
    : transform_(Eigen::Affine3d::Identity()) {
  transform_.translate(position.vector());
  transform_.rotate(orientation.matrix());
}

Transform::Transform(const tf::Transform& st)
    : transform_(Eigen::Affine3d::Identity()) {
  transform_graph::Position position(st.getOrigin());
  transform_graph::Orientation orientation(st.getRotation());
  transform_.translate(position.vector());
  transform_.rotate(orientation.matrix());
}

Transform::Transform(const geometry_msgs::Pose& p)
    : transform_(Eigen::Affine3d::Identity()) {
  transform_graph::Position position(p.position);
  transform_graph::Orientation orientation(p.orientation);
  transform_.translate(position.vector());
  transform_.rotate(orientation.matrix());
}

Transform::Transform(const Eigen::Affine3d& a) : transform_(a) {}

Transform::Transform(const Eigen::Matrix4d& m) : transform_(m) {}

Transform Transform::Identity() {
  Transform transform;
  return transform;
}

Eigen::Matrix4d Transform::matrix() const { return transform_.matrix(); }

Transform Transform::inverse() const {
  return Transform(transform_.inverse(Eigen::Affine).matrix());
}
}  // namespace transform_graph
