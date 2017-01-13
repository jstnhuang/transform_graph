#ifndef _STF_TRANSFORM_H_
#define _STF_TRANSFORM_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

#include "stf/orientation.h"
#include "stf/position.h"

namespace stf {
// Transform describes a combination of a rotation and a translation.
// Its primary uses are as a description of a frame and as a transform mapping.
// As a description of a frame, it gives the origin and orientation of the frame
// relative to a reference frame.
// As a transform mapping between frames A and B, it can take a point in frame B
// (the source frame) and output the coordinates of that same point in terms of
// frame A (the target or reference frame).
class Transform {
 public:
  Transform();
  Transform(const stf::Position& position, const stf::Orientation& orientation);
  Transform(const tf::Transform& st);
  Transform(const geometry_msgs::Pose& p);
  Transform(const Eigen::Affine3d& a);
  Transform(const Eigen::Matrix4d& m);

  static Transform Identity();

  // Returns this transform's homogeneous transform matrix.
  Eigen::Matrix4d matrix() const;

  // Returns the inverse of this transform.
  stf::Transform inverse() const;

 private:
  Eigen::Affine3d transform_;
};
}  // namespace stf

#endif  // _STF_TRANSFORM_H_
