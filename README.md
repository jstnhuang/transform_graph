# stf - Static transformations library

[![Build Status](https://travis-ci.org/jstnhuang/stf.svg?branch=indigo-devel)](https://travis-ci.org/jstnhuang/stf)
[![Coverage Status](https://coveralls.io/repos/github/jstnhuang/stf/badge.svg?branch=indigo-devel)](https://coveralls.io/github/jstnhuang/stf?branch=indigo-devel)

This is a library that computes transformations betweens arbitrary frames in a graph of transformations.
See the generated documentation for details.

Basic example:
```cpp
#include "stf/stf.h"

stf::Graph graph;
graph.Add("wrist", stf::RefFrame("base_link"), wrist_pose_stamped);
graph.Add("kinect", stf::RefFrame("base_link"), kinect_pose_stamped);

// Find out if a point in the frame of the Kinect is near the wrist.
pcl::PointXYZ point_in_kinect = ...;
stf::Position point_in_wrist;
graph.DescribePosition(point_in_kinect, stf::Source("kinect"), stf::Target("wrist"), &point_in_wrist);
if (point_in_wrist.vector().norm() < 0.05) {
  ...
}
```
