#include "stf/graph.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "stf/transform.h"

using std::pair;
using std::string;

typedef pair<string, string> FrameKey;

namespace stf {
Graph::Graph() : transforms_(), graph_() {}

void Graph::Add(const std::string& name, const Source& source,
                const Transform& transform) {
  graph_.AddEdge(source.id(), name);
  transforms_[FrameKey(source.id(), name)] = transform;
}

bool Graph::CanTransform(const string& frame1_id,
                         const string& frame2_id) const {
  std::vector<string> path;
  graph_.Path(frame1_id, frame2_id, &path);
  return path.size() > 0;
}

bool Graph::ComputeTransform(const Source& local_frame,
                             const Target& reference_frame,
                             Transform* transform) const {
  std::vector<string> path;
  graph_.Path(reference_frame.id(), local_frame.id(), &path);

  if (path.size() == 0) {
    return false;
  }

  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  for (size_t i = 1; i < path.size(); ++i) {
    Eigen::Affine3d working = Eigen::Affine3d::Identity();
    bool success = GetTransform(path[i - 1], path[i], &working);
    if (!success) {
      return false;
    }
    result = result * working;
  }
  *transform = result;

  return true;
}

bool Graph::TransformFrame(const Transform& in, const Source& local_frame,
                           const Target& reference_frame,
                           Transform* out) const {
  std::vector<string> path;
  graph_.Path(reference_frame.id(), local_frame.id(), &path);

  if (path.size() == 0) {
    return false;
  }

  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  for (size_t i = 1; i < path.size(); ++i) {
    Eigen::Affine3d working = Eigen::Affine3d::Identity();
    bool success = GetTransform(path[i - 1], path[i], &working);
    if (!success) {
      return false;
    }
    result = result * working;
  }
  result = result * in.matrix();
  *out = result;

  return true;
}

bool Graph::TransformPosition(const Position& in, const Source& local_frame,
                              const Target& reference_frame,
                              Position* out) const {
  std::vector<string> path;
  graph_.Path(reference_frame.id(), local_frame.id(), &path);

  if (path.size() == 0) {
    return false;
  }

  Eigen::Vector3d result = in.vector();
  // Iterate backward to do fewer multiplications.
  for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
    Eigen::Affine3d working = Eigen::Affine3d::Identity();
    bool success = GetTransform(path[i], path[i + 1], &working);
    if (!success) {
      return false;
    }
    result = working * result;
  }
  *out = result;

  return true;
}

bool Graph::GetTransform(const std::string& source, const std::string& target,
                         Eigen::Affine3d* out) const {
  FrameKey key(source, target);
  FrameKey inverse_key(target, source);
  std::map<FrameKey, Transform>::const_iterator it = transforms_.find(key);
  if (it != transforms_.end()) {
    *out = it->second.matrix();
    return true;
  } else {
    it = transforms_.find(inverse_key);
    if (it != transforms_.end()) {
      *out = it->second.inverse().matrix();
      return true;
    }
  }
  return false;
}
}  // namespace stf
