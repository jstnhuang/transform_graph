stf - Static Transforms Library {#mainpage}
===============================

stf is a library for computing transformations between coordinate frames. It is similar to the tf library but is more lightweight.

## Differences from tf

- Coordinate frames do not have to be structured in a tree. Instead, frames can be arranged in an arbitrary graph (weakly connected, cyclic, disconnected).
- stf does not depend on ROS except to convert from common message types. As a result, you do not need to run a ROS master to use stf and it is suitable for use in pure unit tests.
- stf is not a distributed system. Programmers simply create and use the stf graph as an object in memory.

## Quick start

stf::Graph maintains the graph of transformations and is the primary interface to stf:
~~~{.cpp}
#include "stf/stf.h"

int main(int argc, char** argv) {
  stf::Graph graph; 
  return 0;
}
~~~

Add frames to the graph using stf::Graph::Add:
~~~{.cpp}
stf::Graph graph;

geometry_msgs::Pose torso_pose;
pose.position.z = 0.4;
pose.orientation.w = 1;

graph.Add("torso_lift_link", stf::RefFrame("base_link"), torso_pose);
~~~

Get points in different frames using stf::Graph::DescribePosition.
In this example, we want to know what a point 10 cm in front of the robot's wrist is, expressed in the base frame:
~~~{.cpp}
geometry_msgs::Point pt;
pt.x = 0.1;
stf::Transform pt_in_base;
graph.DescribePosition(pt, stf::Source("wrist"), stf::Target("base_link"), &pt_in_base);
Eigen::Vector3d v = pt_in_base.vector();
~~~

## Types used by stf

stf::Transform is the library's representation of a transformation.
Similarly, stf::Position and stf::Orientation describe positions and orientations.
Many common types, (e.g., geometry_msgs::Point, Eigen::Quaterniond, geometry_msgs::Pose, pcl::PointXYZ) can be implicitly converted into their stf equivalents.
See transform.h for a list of types that can be implicitly converted.

Currently, the library does not have functions to convert from stf types back into common types.
Instead, call stf::Position::vector(), stf::Orientation::matrix(), or stf::Transform::matrix(), which return Eigen::Vector3d, Eigen::Matrix3d (a rotation matrix), and Eigen::Matrix4d (a homogeneous transform matrix), respectively.

Types like stf::Source, stf::RefFrame, stf::From, etc. are wrappers around strings and are intended to help clarify the meaning of function arguments.

## Describing vs. Mapping
stf supports two operations, *describing* and *mapping*, which are different ways of thinking about the same thing.

To *describe* a point is to express the coordinates of the point in a different frame of reference, called the target frame.
For example, suppose there is a left wrist frame at (0, 0.3, 0) and a right wrist frame at (0, -0.3, 0) and they have the same orientation.
The position of the right wrist is (0, 0, 0) in the right wrist frame but can be described in the frame of the left wrist as (0, -0.6, 0).
Similarly, the point (0.1, 0, 0) in the right wrist frame can be written as (0.1, -0.6, 0) in the left wrist frame.

To *map* a vector is to "move" a vector into a new frame, instead of looking at the same vector in a different reference frame.
Another way to think about it is applying the same rotation and translation to the vector that it took to move between the two frames.
For example, using the same setup as above, you can "move" the point (0.1, 0, 0) from the left wrist frame into the right wrist frame.
When described in the left wrist frame, the mapped point is (0.1, -0.6, 0).

The equivalence is that to map a vector *from* the left *to* the right is the same as having the vector be located in the right frame to start with (the *source*) and to describe it in the *target* frame of the left.
Most of the time, it's easier to think about the problem as "describing," but it can occasionally be helpful to think about it as "mapping" instead.
Below is a summary of frame relationships:

Usage | Reference frame | Local frame
----- | --------------- | -----------
General | stf::RefFrame | stf::LocalFrame
Description | stf::Target | stf::Source
Mapping | stf::To | stf::From

## ComputeDescription vs. DescribePosition
Each time you call stf::Graph::DescribePosition or stf::Graph::MapPosition, stf finds a path of transformations between the frames and multiplies the point through the chain from right to left, which leads to fewer multiplications than doing so left to right.
However, if you are going to transform many points between the same two frames, it will be more efficient to just multiply all of the matrices along the transformation chain once and use the resulting matrix over and over.
To do so, call stf::Graph::ComputeDescription.
For convenience, you can always place the result back into the graph.
stf will always use this transform because it will be the shortest path between the two frames.

For example, suppose we want to transform thousands of points in a point cloud into the wrist frame.
~~~{.cpp}
// Compute the frame describing the Kinect frame relative to the wrist frame.
stf::Transform cloud_in_wrist;
graph.ComputeDescription(stf::LocalFrame("head_mount_kinect"), stf::RefFrame("wrist"), &cloud_in_wrist);

// Add it back into the graph.
graph.Add("head_mount_kinect", stf::RefFrame("wrist"), cloud_in_wrist);

for (size_t i=0; i<cloud.size(); ++i) {
  const pcl::PointXYZ& pt = cloud.points[i];
  stf::Position pt_in_wrist;
  graph.DescribePosition(pt, stf::Source("head_mount_kinect"), stf::Target("wrist"), &pt_in_wrist);
}
~~~

## Graph details
- If there are multiple paths between two frames, the shortest one is chosen.
- You can update a transform by simply adding it again.
- You do not need to add the inverse of a transform, stf automatically computes them. If you do add the inverse of an existing edge, it will update the edge in the graph.
