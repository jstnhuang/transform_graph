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
  //
  // Example (adds a tooltip that is 10cm in front of the wrist):
  //  geometry_msgs::Pose pose;
  //  pose.position.x = 0.1;
  //  pose.orientation.w = 1;
  //  graph.Add("tooltip", stf::RefFrame("wrist"), pose);
  void Add(const std::string& name, const RefFrame& ref_frame_name,
           const Transform& transform);

  // Returns true if an undirected path exists between the given frames and
  // false otherwise.
  bool CanTransform(const std::string& frame1_id,
                    const std::string& frame2_id) const;

  // Computes the transform describing the local frame relative to the reference
  // frame. Multiplying this transform with a point expressed in the local frame
  // results in the same point expressed in the reference frame.
  //
  // Returns false if a transform couldn't be calculated.
  //
  // Example (describes the wrist frame relative to the base frame):
  //  graph.ComputeDescription(stf::LocalFrame("wrist"), stf::RefFrame("base"),
  //    &wrist_in_base);
  //
  // The origin of the wrist in the base frame is
  //   wrist_in_base.position().vector().
  bool ComputeDescription(const LocalFrame& local, const RefFrame& reference,
                          Transform* transform) const;

  // Computes the transform that maps points in the "from" frame to the "to"
  // frame frame. To do the mapping, multiply the point with this transform.
  //
  // Returns false if a transform couldn't be calculated.
  //
  // Example (get a point in front of the right hand):
  //  graph.Add("left hand", stf::RefFrame("base"), ...);
  //  graph.Add("right hand", stf::RefFrame("base"), ...);
  //  graph.ComputeMapping(stf::From("left hand"), stf::To("right hand"),
  //    &left_to_right);
  // Now multiplying left_to_right by a point 10 cm in front of the left hand
  // results in a point 10 cm in front of the right hand. However, this point is
  // still expressed in the frame of the left hand.
  //
  // Note that this is the same as ComputeDescription, but with the frame order
  // inverted.
  bool ComputeMapping(const From& from, const To& to,
                      Transform* transform) const;

  // Given a pose in some source frame, describe that same pose in the target
  // frame. This is similar to simply adding a new frame to the graph and
  // calling ComputeDescription.
  //
  // Example: get the frame 10 cm in front of the wrist in base coordinates:
  //  Pose p;
  //  p.position.x = 0.1;
  //  p.orientation.w = 1;
  //  graph.TransformFrame(p, stf::Source("wrist"), stf::Target("base"),
  //    &ps_base);
  bool DescribePose(const Transform& in, const Source& source_frame,
                    const Target& target_frame, Transform* out) const;

  // Given a pose, map that pose from the "from" frame to the "to" frame. I.e.,
  // apply the same rotation and translation to go from "from" to "to" to the
  // given pose. The output pose is still in the frame of "from".
  bool MapPose(const Transform& in, const From& frame_in, const To& frame_out,
               Transform* out) const;

  // Given a position described in the the source frame (frame_in), output the
  // same position as described in the target frame (frame_out).
  //
  // Example (get the position 10 cm in front of the wrist in the base frame):
  //  PointStamped ps;
  //  ps.header.frame_id = "wrist";
  //  ps.pose.position.x = 0.1;
  //  graph.DescribePosition(ps.point, stf::Source("wrist"),
  //    stf::Target("base"), &ps_base)
  bool DescribePosition(const Position& in, const Source& source_frame,
                        const Target& target_frame, Position* out) const;

  // Given a position expressed in the "from" frame, map that position into the
  // "to" frame, i.e., apply the same rotation and offset to go from
  // "from"->"to" to the vector. The resulting position is still expressed in
  // the "from" frame.
  //
  // Example (get a point in front of the right hand):
  // Suppose the left hand and right hand have the same orientation and differ
  // only in y value, with the left hand at +0.1 and the right hand being at
  // -0.1.
  //
  // If p_in is (1, 0, 0), then this results in p_out=(1, -0.2, 0):
  //  graph.MapPosition(p, stf::From("left hand"), stf::To("right hand"),
  //    &p_out);
  bool MapPosition(const Position& in, const From& frame_in,
                   const To& frame_out, Position* out) const;

 private:
  bool GetTransform(const std::string& source, const std::string& target,
                    Eigen::Affine3d* out) const;

  std::map<std::pair<std::string, std::string>, Transform> transforms_;
  internal::Graph graph_;
};
}  // namespace stf

#endif  // _STF_GRAPH_H_
