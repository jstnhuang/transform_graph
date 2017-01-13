#ifndef _STF_GRAPH_H_
#define _STF_GRAPH_H_

#include <map>
#include <string>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "stf/explicit_types.h"
#include "stf/graph_internal.h"
#include "stf/transform.h"

namespace stf {
// Graph models static transformations between coordinate frames.
//
// The vertices are coordinate frame IDs, given as strings. The edges are
// directed and represent transformations between coordinate
// frames.
//
// Once the graph is specified, this class can be used to compute the
// transformation between any two coordinate frames, as long as a weakly
// connected path exists between them.
//
// The graph may contain cycles. If it does, our shortest path
// algorithm will find a path with the minimum number of edges.
//
// The graph may be disconnected, in which case Graph may be used to find
// transformations between two frames in the same connected component.
class Graph {
 public:
  Graph();

  // Adds a transform (an edge) to the graph.
  // See transform.h to see what types will be implicitly converted to an
  // stf::Transform.
  // This includes tf::Transform, so you can import a transform from tf like so:
  // tf::StampedTransform tf_stamped;
  // tf_listener.lookupTransform("base_link", "head_mount_kinect_link",
  // ros::Time(0), tf_stamped);
  // graph.Add(tf_stamped);
  void Add(const std::string& name, const Source& source,
           const Transform& transform);

  // Returns true if an undirected path exists between the given frames and
  // false otherwise.
  bool CanTransform(const std::string& frame1_id,
                    const std::string& frame2_id) const;

  // Computes the transform describing the local (source) frame relative to
  // the reference (target) frame.
  // Note that multiplying this transform with a point expressed in the source
  // frame results in the same point expressed in target.
  //
  // Returns false if a transform couldn't be calculated.
  //
  // Example (gets transform that describes the wrist frame relative to the base
  // frame):
  //  graph.ComputeTransform(stf::Source("wrist"), stf::Target("base"),
  //    &wrist_in_base);
  // The origin of the wrist in the base frame is
  //   wrist_in_base.position().vector().
  bool ComputeTransform(const Source& local_frame,
                        const Target& reference_frame,
                        Transform* transform) const;

  // Example: get the frame 10 cm in front of the wrist in base coordinates:
  //  PoseStamped ps;
  //  ps.header.frame_id = "wrist";
  //  ps.pose.position.x = 0.1;
  //  graph.TransformFrame(ps.pose, stf::Source("wrist"), stf::Target("base"),
  //    &ps_base)
  bool TransformFrame(const Transform& in, const Source& local_frame,
                      const Target& reference_frame, Transform* out) const;

  // Example:
  //  PointStamped ps;
  //  ps.header.frame_id = "wrist";
  //  ps.pose.position.x = 0.1;
  //  graph.TransformPosition(ps.point, stf::Source("wrist"),
  //    stf::Target("base"), &ps_base)
  bool TransformPosition(const Position& in, const Source& local_frame,
                         const Target& reference_frame, Position* out) const;

 private:
  bool GetTransform(const std::string& source, const std::string& target,
                    Eigen::Affine3d* out) const;

  std::map<std::pair<std::string, std::string>, Transform> transforms_;
  internal::Graph graph_;
};
}  // namespace stf

#endif  // _STF_GRAPH_H_
