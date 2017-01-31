#ifndef _STF_POSITION_H_
#define _STF_POSITION_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"

namespace stf {
// Position provides conversions from common position types.
class Position {
 public:
  Position();
  Position(double x, double y, double z);
  Position(const Eigen::Vector3d& v);
  Position(const geometry_msgs::Point& p);
  Position(const geometry_msgs::Vector3& v);
  Position(const pcl::PointXYZ& p);
  Position(const tf::Vector3& v);

  // TODO: Add more PCL point types.

  // Returns this translation as an Eigen::Vector3d.
  Eigen::Vector3d vector() const;

 private:
  Eigen::Vector3d vector_;
};
}  // namespace stf

#endif  // _STF_POSITION_H_
