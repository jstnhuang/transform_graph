#include "transform_graph/transform.h"

#include "Eigen/Dense"

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

#include "transform_graph/orientation.h"
#include "transform_graph/position.h"

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
  tf::poseMsgToEigen(p, transform_);
}

Transform::Transform(const geometry_msgs::Transform& p)
    : transform_(Eigen::Affine3d::Identity()) {
  tf::transformMsgToEigen(p, transform_);
}

Transform::Transform(const Eigen::Affine3d& a) : transform_(a) {}

Transform::Transform(const Eigen::Matrix4d& m) : transform_(m) {}

Transform Transform::Identity() {
  Transform transform;
  return transform;
}

transform_graph::Position Transform::position() const {
  transform_graph::Position position(transform_.translation());
  return position;
}

transform_graph::Orientation Transform::orientation() const {
  transform_graph::Orientation orientation(transform_.rotation());
  return orientation;
}

tf::Transform Transform::tf_transform() const {
  tf::Transform tft;
  tf::transformEigenToTF(transform_, tft);
  return tft;
}

geometry_msgs::Pose Transform::pose() const {
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(transform_, pose);
  return pose;
}

geometry_msgs::Transform Transform::transform() const {
  geometry_msgs::Transform transform;
  tf::transformEigenToMsg(transform_, transform);
  return transform;
}

Eigen::Matrix4d Transform::matrix() const { return transform_.matrix(); }

Eigen::Affine3d Transform::affine() const { return transform_; }

void Transform::ToPose(geometry_msgs::Pose* pose) const {
  pose->position.x = transform_.translation().x();
  pose->position.y = transform_.translation().y();
  pose->position.z = transform_.translation().z();
  Eigen::Quaterniond q(transform_.rotation());
  pose->orientation.w = q.w();
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
}

Transform Transform::inverse() const {
  return Transform(transform_.inverse(Eigen::Affine).matrix());
}
}  // namespace transform_graph
