#include "stf/transform.h"

#include "Eigen/Dense"

#include "stf/position.h"
#include "stf/orientation.h"

using std::string;

namespace stf {
Transform::Transform() : transform_(Eigen::Affine3d::Identity()) {}
Transform::Transform(const stf::Position& position,
                     const stf::Orientation& orientation)
    : transform_(Eigen::Affine3d::Identity()) {
  transform_.translate(position.vector());
  transform_.rotate(orientation.matrix());
}

Transform::Transform(const tf::Transform& st)
    : transform_(Eigen::Affine3d::Identity()) {
  stf::Position position(st.getOrigin());
  stf::Orientation orientation(st.getRotation());
  transform_.translate(position.vector());
  transform_.rotate(orientation.matrix());
}

Transform::Transform(const geometry_msgs::Pose& p)
    : transform_(Eigen::Affine3d::Identity()) {
  stf::Position position(p.position);
  stf::Orientation orientation(p.orientation);
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
}  // namespace stf
